import simpy
import numpy as np
import random
import math
import queue
from simulator.log import logger
from entities.packet import DataPacket
from routing.dsdv.dsdv import Dsdv
from mac.tdma import Tdma
from mac.csma_ca import CsmaCa # Assuming CSMA implementation exists
from mobility.gauss_markov_3d import GaussMarkov3D
from mobility.circular_pattern import CircularPattern  # New mobility pattern
from energy.energy_model import EnergyModel
from allocation.channel_assignment import ChannelAssigner
from utils import config
from utils.util_function import has_intersection
from phy.large_scale_fading import sinr_calculator


class Drone:
    """
    Enhanced Dynamic Drone with Adaptive Behaviors
    
    New Features:
    1. Dynamic mobility: Switches between circular and random movement patterns
    2. Obstacle avoidance in dynamic environments
    3. Adaptive MAC protocol switching (TDMA ↔ CSMA)
    4. Real-time performance monitoring and metrics collection
    5. Contention-based protocol selection
    
    Author: Enhanced version for dynamic behavior
    Created at: 2025/1/13
    """

    def __init__(self,
                 env,
                 node_id,
                 coords,
                 speed,
                 inbox,
                 simulator):
        # ============= Original Attributes =============
        self.simulator = simulator
        self.env = env
        self.identifier = node_id
        self.coords = coords
        self.start_coords = coords

        self.rng_drone = random.Random(self.identifier + self.simulator.seed)

        self.direction = self.rng_drone.uniform(0, 2 * np.pi)
        self.pitch = self.rng_drone.uniform(-0.05, 0.05)
        self.speed = speed
        self.velocity = [self.speed * math.cos(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.pitch)]

        self.direction_mean = self.direction
        self.pitch_mean = self.pitch
        self.velocity_mean = self.speed

        self.inbox = inbox
        self.buffer = simpy.Resource(env, capacity=1)
        self.max_queue_size = config.MAX_QUEUE_SIZE
        self.transmitting_queue = queue.Queue()
        self.waiting_list = []

        # ============= NEW: Dual MAC Protocol Support =============
        self.mac_tdma = Tdma(self)
        self.mac_csma = CsmaCa(self)
        self.mac_protocol = self.mac_tdma  # Start with TDMA
        self.current_mac_type = "TDMA"
        
        self.mac_process_dict = dict()
        self.mac_process_finish = dict()
        self.mac_process_count = 0
        self.enable_blocking = 1

        self.routing_protocol = Dsdv(self.simulator, self)

        # ============= NEW: Dual Mobility Model Support =============
        self.mobility_gauss_markov = GaussMarkov3D(self)
        self.mobility_circular = CircularPattern(self)
        self.mobility_model = self.mobility_gauss_markov  # Start with random
        self.current_mobility_type = "RANDOM"
        
        self.energy_model = EnergyModel(self)
        self.residual_energy = config.INITIAL_ENERGY
        self.sleep = False

        self.channel_assigner = ChannelAssigner(self.simulator, self)

        # ============= NEW: Dynamic Behavior Attributes =============
        
        # Mobility switching parameters
        self.mobility_switch_interval = 30 * 1e6  # Switch every 30 seconds
        self.last_mobility_switch_time = 0
        self.circular_pattern_duration = 15 * 1e6  # Stay circular for 15s
        self.in_circular_mode = False
        self.circular_center = None
        self.circular_radius = 50  # meters
        
        # Obstacle avoidance parameters
        self.obstacle_detection_range = 30  # meters
        self.obstacle_avoidance_active = False
        self.avoidance_direction = None
        self.min_obstacle_distance = 10  # minimum safe distance
        
        # MAC protocol switching parameters
        self.mac_switch_interval = 20 * 1e6  # Evaluate every 20 seconds
        self.last_mac_switch_time = 0
        self.contention_window = 5 * 1e6  # 5 second window for measuring contention
        self.collision_count = 0
        self.successful_tx_count = 0
        self.failed_tx_count = 0
        
        # Performance metrics per MAC protocol
        self.metrics_tdma = {
            'pdr': [],           # Packet Delivery Ratio
            'throughput': [],    # bits per second
            'delay': [],         # end-to-end delay
            'energy': [],        # energy consumption
            'collisions': [],    # collision count
            'contention_level': []
        }
        
        self.metrics_csma = {
            'pdr': [],
            'throughput': [],
            'delay': [],
            'energy': [],
            'collisions': [],
            'contention_level': []
        }
        
        # Current measurement window
        self.window_start_time = 0
        self.window_packets_sent = 0
        self.window_packets_received = 0
        self.window_total_delay = 0
        self.window_energy_consumed = 0
        self.window_collisions = 0
        
        # Contention measurement
        self.recent_transmissions = []  # List of recent tx attempts
        self.contention_threshold_high = 0.7  # 70% collision rate = high contention
        self.contention_threshold_low = 0.3   # 30% collision rate = low contention

        # ============= Start Processes =============
        self.env.process(self.generate_data_packet())
        self.env.process(self.feed_packet())
        self.env.process(self.receive())
        
        # ============= NEW: Dynamic Behavior Processes =============
        self.env.process(self.dynamic_mobility_controller())
        self.env.process(self.obstacle_detection_and_avoidance())
        self.env.process(self.adaptive_mac_controller())
        self.env.process(self.performance_monitor())

    # ============================================================================
    # NEW FEATURE 1: Dynamic Mobility Pattern Switching
    # ============================================================================
    
    def dynamic_mobility_controller(self):
        """
        Controls switching between circular and random movement patterns.
        Pattern: Random → Circular → Random → Circular ...
        """
        while True:
            if not self.sleep:
                yield self.env.timeout(self.mobility_switch_interval)
                
                if self.current_mobility_type == "RANDOM":
                    # Switch to circular pattern
                    self.switch_to_circular_mobility()
                    logger.info('At time: %s (us) ---- UAV: %s switches to CIRCULAR mobility pattern',
                                self.env.now, self.identifier)
                    
                    # Stay in circular mode for specified duration
                    yield self.env.timeout(self.circular_pattern_duration)
                    
                else:
                    # Switch back to random pattern
                    self.switch_to_random_mobility()
                    logger.info('At time: %s (us) ---- UAV: %s switches to RANDOM mobility pattern',
                                self.env.now, self.identifier)
            else:
                break
    
    def switch_to_circular_mobility(self):
        """Switch to circular movement pattern"""
        self.current_mobility_type = "CIRCULAR"
        self.in_circular_mode = True
        
        # Set circular pattern center as current position
        self.circular_center = self.coords.copy()
        self.mobility_model = self.mobility_circular
        
        # Initialize circular pattern
        self.mobility_circular.set_center(self.circular_center)
        self.mobility_circular.set_radius(self.circular_radius)
        self.mobility_circular.initialize_circular_motion()
    
    def switch_to_random_mobility(self):
        """Switch to random (Gauss-Markov) movement pattern"""
        self.current_mobility_type = "RANDOM"
        self.in_circular_mode = False
        self.mobility_model = self.mobility_gauss_markov

    # ============================================================================
    # NEW FEATURE 2: Obstacle Detection and Avoidance
    # ============================================================================
    
    def obstacle_detection_and_avoidance(self):
        """
        Continuously monitors for obstacles and performs avoidance maneuvers.
        Checks environment every 0.1 seconds.
        """
        while True:
            if not self.sleep:
                yield self.env.timeout(100000)  # Check every 0.1 seconds
                
                # Detect obstacles in the environment
                obstacles = self.detect_obstacles()
                
                if obstacles:
                    # Calculate safe direction
                    safe_direction = self.calculate_avoidance_direction(obstacles)
                    
                    if safe_direction is not None:
                        self.obstacle_avoidance_active = True
                        self.perform_avoidance_maneuver(safe_direction)
                        
                        logger.info('At time: %s (us) ---- UAV: %s performing obstacle avoidance',
                                    self.env.now, self.identifier)
                else:
                    self.obstacle_avoidance_active = False
            else:
                break
    
    def detect_obstacles(self):
        """
        Detects obstacles within detection range.
        Obstacles can be: other drones, static obstacles, dynamic obstacles
        
        Returns:
            list: List of detected obstacles with positions and types
        """
        obstacles = []
        
        # Check for other drones (potential collision)
        for drone in self.simulator.drones:
            if drone.identifier != self.identifier and not drone.sleep:
                distance = self.calculate_3d_distance(self.coords, drone.coords)
                
                if distance < self.obstacle_detection_range:
                    obstacles.append({
                        'type': 'drone',
                        'position': drone.coords,
                        'velocity': drone.velocity,
                        'distance': distance,
                        'id': drone.identifier
                    })
        
        # Check for static obstacles (if environment has them)
        if hasattr(self.simulator, 'static_obstacles'):
            for obstacle in self.simulator.static_obstacles:
                distance = self.calculate_3d_distance(self.coords, obstacle['position'])
                
                if distance < self.obstacle_detection_range:
                    obstacles.append({
                        'type': 'static',
                        'position': obstacle['position'],
                        'radius': obstacle.get('radius', 5),
                        'distance': distance
                    })
        
        # Check for dynamic obstacles (moving objects)
        if hasattr(self.simulator, 'dynamic_obstacles'):
            for obstacle in self.simulator.dynamic_obstacles:
                distance = self.calculate_3d_distance(self.coords, obstacle['position'])
                
                if distance < self.obstacle_detection_range:
                    obstacles.append({
                        'type': 'dynamic',
                        'position': obstacle['position'],
                        'velocity': obstacle.get('velocity', [0, 0, 0]),
                        'distance': distance
                    })
        
        return obstacles
    
    def calculate_avoidance_direction(self, obstacles):
        """
        Calculates safe direction to avoid obstacles using potential field method.
        
        Parameters:
            obstacles: List of detected obstacles
            
        Returns:
            tuple: (new_direction, new_pitch) or None if no safe direction
        """
        # Repulsive force from obstacles
        repulsive_force = np.array([0.0, 0.0, 0.0])
        
        for obstacle in obstacles:
            # Vector from obstacle to drone
            diff = np.array(self.coords) - np.array(obstacle['position'])
            distance = obstacle['distance']
            
            if distance < self.min_obstacle_distance:
                # Strong repulsion for very close obstacles
                magnitude = 1000 / (distance + 0.1)
            else:
                magnitude = 100 / (distance + 1)
            
            # Normalize and scale
            if np.linalg.norm(diff) > 0:
                repulsive_force += (diff / np.linalg.norm(diff)) * magnitude
        
        # Attractive force toward goal (original direction)
        original_velocity = np.array(self.velocity)
        attractive_force = original_velocity * 10
        
        # Combined force
        total_force = attractive_force + repulsive_force
        
        if np.linalg.norm(total_force) > 0:
            # Calculate new direction and pitch
            new_direction = math.atan2(total_force[1], total_force[0])
            horizontal_magnitude = math.sqrt(total_force[0]**2 + total_force[1]**2)
            new_pitch = math.atan2(total_force[2], horizontal_magnitude)
            
            # Limit pitch to safe values
            new_pitch = max(-0.3, min(0.3, new_pitch))
            
            return (new_direction, new_pitch)
        
        return None
    
    def perform_avoidance_maneuver(self, safe_direction):
        """
        Executes avoidance maneuver by adjusting drone's direction and pitch.
        
        Parameters:
            safe_direction: tuple of (direction, pitch)
        """
        new_direction, new_pitch = safe_direction
        
        # Smoothly transition to new direction
        self.direction = new_direction
        self.pitch = new_pitch
        
        # Update velocity
        self.velocity = [
            self.speed * math.cos(self.direction) * math.cos(self.pitch),
            self.speed * math.sin(self.direction) * math.cos(self.pitch),
            self.speed * math.sin(self.pitch)
        ]
        
        # Update mean values for mobility model
        self.direction_mean = self.direction
        self.pitch_mean = self.pitch
    
    def calculate_3d_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two 3D positions"""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))

    # ============================================================================
    # NEW FEATURE 3: Adaptive MAC Protocol Switching
    # ============================================================================
    
    def adaptive_mac_controller(self):
        """
        Monitors network contention and switches between TDMA and CSMA.
        
        Strategy:
        - High contention (many drones transmitting) → TDMA (scheduled access)
        - Low contention (few drones transmitting) → CSMA (random access)
        """
        while True:
            if not self.sleep:
                yield self.env.timeout(self.mac_switch_interval)
                
                # Measure current contention level
                contention_level = self.measure_contention()
                
                logger.info('At time: %s (us) ---- UAV: %s measured contention: %.3f',
                            self.env.now, self.identifier, contention_level)
                
                # Decision logic
                if contention_level >= self.contention_threshold_high:
                    # High contention → Switch to TDMA
                    if self.current_mac_type != "TDMA":
                        self.switch_to_tdma()
                        logger.info('At time: %s (us) ---- UAV: %s switches to TDMA (high contention: %.3f)',
                                    self.env.now, self.identifier, contention_level)
                
                elif contention_level <= self.contention_threshold_low:
                    # Low contention → Switch to CSMA
                    if self.current_mac_type != "CSMA":
                        self.switch_to_csma()
                        logger.info('At time: %s (us) ---- UAV: %s switches to CSMA (low contention: %.3f)',
                                    self.env.now, self.identifier, contention_level)
                
                # Record contention level in metrics
                current_metrics = self.get_current_metrics_dict()
                current_metrics['contention_level'].append(contention_level)
            else:
                break
    
    def measure_contention(self):
        """
        Measures network contention level based on collision rate.
        
        Returns:
            float: Contention level (0.0 to 1.0)
        """
        # Clean old transmission records
        current_time = self.env.now
        self.recent_transmissions = [
            tx for tx in self.recent_transmissions 
            if current_time - tx['time'] < self.contention_window
        ]
        
        if len(self.recent_transmissions) == 0:
            return 0.0  # No recent activity
        
        # Calculate collision rate
        collisions = sum(1 for tx in self.recent_transmissions if not tx['success'])
        total = len(self.recent_transmissions)
        
        collision_rate = collisions / total if total > 0 else 0
        
        # Also consider number of nearby active drones
        nearby_drones = self.count_nearby_active_drones(radius=100)  # within 100m
        drone_density_factor = min(nearby_drones / config.NUMBER_OF_DRONES, 1.0)
        
        # Combined contention metric
        contention = 0.7 * collision_rate + 0.3 * drone_density_factor
        
        return contention
    
    def count_nearby_active_drones(self, radius):
        """Count number of active drones within specified radius"""
        count = 0
        for drone in self.simulator.drones:
            if drone.identifier != self.identifier and not drone.sleep:
                distance = self.calculate_3d_distance(self.coords, drone.coords)
                if distance <= radius:
                    count += 1
        return count
    
    def switch_to_tdma(self):
        """Switch MAC protocol to TDMA"""
        # Record current CSMA metrics before switching
        self.record_current_window_metrics()
        
        self.mac_protocol = self.mac_tdma
        self.current_mac_type = "TDMA"
        self.last_mac_switch_time = self.env.now
        
        # Reset measurement window
        self.reset_measurement_window()
    
    def switch_to_csma(self):
        """Switch MAC protocol to CSMA"""
        # Record current TDMA metrics before switching
        self.record_current_window_metrics()
        
        self.mac_protocol = self.mac_csma
        self.current_mac_type = "CSMA"
        self.last_mac_switch_time = self.env.now
        
        # Reset measurement window
        self.reset_measurement_window()
    
    def get_current_metrics_dict(self):
        """Returns the appropriate metrics dictionary based on current MAC"""
        if self.current_mac_type == "TDMA":
            return self.metrics_tdma
        else:
            return self.metrics_csma

    # ============================================================================
    # NEW FEATURE 4: Performance Monitoring and Metrics Collection
    # ============================================================================
    
    def performance_monitor(self):
        """
        Continuously monitors and records performance metrics.
        Updates metrics every second.
        """
        while True:
            if not self.sleep:
                yield self.env.timeout(1000000)  # Every 1 second
                
                # Calculate current window metrics
                self.calculate_and_record_metrics()
                
                # Log current performance
                if self.env.now % 10000000 == 0:  # Every 10 seconds
                    self.log_performance_summary()
            else:
                break
    
    def calculate_and_record_metrics(self):
        """Calculate metrics for current measurement window"""
        window_duration = self.env.now - self.window_start_time
        
        if window_duration == 0:
            return
        
        # PDR (Packet Delivery Ratio)
        pdr = (self.window_packets_received / self.window_packets_sent 
               if self.window_packets_sent > 0 else 0)
        
        # Throughput (bits per second)
        payload_length = getattr(config, 'AVERAGE_PAYLOAD_LENGTH', 8192)
        throughput = (self.window_packets_received * payload_length * 1e6 
                     / window_duration if window_duration > 0 else 0)
        
        # Average delay (microseconds)
        avg_delay = (self.window_total_delay / self.window_packets_received 
                    if self.window_packets_received > 0 else 0)
        
        # Energy consumption rate (Joules per second)
        energy_rate = self.window_energy_consumed * 1e6 / window_duration if window_duration > 0 else 0
        
        # Get current metrics dictionary
        metrics = self.get_current_metrics_dict()
        
        # Record metrics (only if we have valid data)
        if self.window_packets_sent > 0 or self.window_packets_received > 0:
            metrics['pdr'].append(pdr)
            metrics['throughput'].append(throughput)
            metrics['delay'].append(avg_delay)
            metrics['energy'].append(energy_rate)
            metrics['collisions'].append(self.window_collisions)
    
    def record_current_window_metrics(self):
        """Records metrics when switching protocols"""
        self.calculate_and_record_metrics()
    
    def reset_measurement_window(self):
        """Reset measurement window counters"""
        self.window_start_time = self.env.now
        self.window_packets_sent = 0
        self.window_packets_received = 0
        self.window_total_delay = 0
        self.window_energy_consumed = 0
        self.window_collisions = 0
    
    def log_performance_summary(self):
        """Log performance summary for both protocols"""
        logger.info('=' * 80)
        logger.info('Performance Summary at time %s (us) for UAV: %s', self.env.now, self.identifier)
        logger.info('Current MAC: %s, Current Mobility: %s', self.current_mac_type, self.current_mobility_type)
        logger.info('-' * 80)
        
        # TDMA metrics
        if self.metrics_tdma['pdr']:
            logger.info('TDMA - PDR: %.3f, Throughput: %.2f bps, Delay: %.2f us, Collisions: %d',
                        np.mean(self.metrics_tdma['pdr']),
                        np.mean(self.metrics_tdma['throughput']),
                        np.mean(self.metrics_tdma['delay']),
                        sum(self.metrics_tdma['collisions']))
        
        # CSMA metrics
        if self.metrics_csma['pdr']:
            logger.info('CSMA - PDR: %.3f, Throughput: %.2f bps, Delay: %.2f us, Collisions: %d',
                        np.mean(self.metrics_csma['pdr']),
                        np.mean(self.metrics_csma['throughput']),
                        np.mean(self.metrics_csma['delay']),
                        sum(self.metrics_csma['collisions']))
        
        logger.info('Residual Energy: %.2f J', self.residual_energy)
        logger.info('=' * 80)
    
    def record_transmission_attempt(self, success):
        """Record a transmission attempt for contention measurement"""
        self.recent_transmissions.append({
            'time': self.env.now,
            'success': success
        })
        
        self.window_packets_sent += 1
        
        if success:
            self.successful_tx_count += 1
        else:
            self.failed_tx_count += 1
            self.window_collisions += 1
    
    def record_packet_reception(self, packet):
        """Record successful packet reception"""
        self.window_packets_received += 1
        
        # Calculate delay (only for data packets with creation_time)
        if hasattr(packet, 'creation_time') and packet.creation_time is not None:
            delay = self.env.now - packet.creation_time
            self.window_total_delay += delay

    # ============================================================================
    # ORIGINAL FUNCTIONS (Modified to support new features)
    # ============================================================================

    def generate_data_packet(self, traffic_pattern='Poisson'):
        """
        Generate data packets (original function with metrics tracking)
        """
        while True:
            if not self.sleep:
                if traffic_pattern == 'Uniform':
                    yield self.env.timeout(self.rng_drone.randint(500000, 505000))
                elif traffic_pattern == 'Poisson':
                    rate = 5
                    yield self.env.timeout(round(self.rng_drone.expovariate(rate) * 1e6))

                config.GL_ID_DATA_PACKET += 1

                all_candidate_list = [i for i in range(config.NUMBER_OF_DRONES)]
                all_candidate_list.remove(self.identifier)
                dst_id = self.rng_drone.choice(all_candidate_list)
                destination = self.simulator.drones[dst_id]

                # Safely get payload length with defaults
                variable_payload = getattr(config, 'VARIABLE_PAYLOAD_LENGTH', False)
                avg_payload = getattr(config, 'AVERAGE_PAYLOAD_LENGTH', 8192)
                max_variation = getattr(config, 'MAXIMUM_PAYLOAD_VARIATION', 1024)
                
                if variable_payload:
                    fluctuation = self.rng_drone.randint(-max_variation, max_variation)
                    payload_length = avg_payload + fluctuation
                else:
                    payload_length = avg_payload

                # Safely get header lengths with defaults
                ip_header = getattr(config, 'IP_HEADER_LENGTH', 160)
                mac_header = getattr(config, 'MAC_HEADER_LENGTH', 272)
                phy_header = getattr(config, 'PHY_HEADER_LENGTH', 128)
                
                data_packet_length = ip_header + mac_header + phy_header + payload_length

                channel_id = self.channel_assigner.channel_assign()

                pkd = DataPacket(self,
                                 dst_drone=destination,
                                 creation_time=self.env.now,
                                 data_packet_id=config.GL_ID_DATA_PACKET,
                                 data_packet_length=data_packet_length,
                                 simulator=self.simulator,
                                 channel_id=channel_id)
                pkd.transmission_mode = 0

                self.simulator.metrics.datapacket_generated_num += 1

                logger.info('At time: %s (us) ++++ UAV: %s generates packet (id: %s, dst: %s) using %s',
                            self.env.now, self.identifier, pkd.packet_id, 
                            destination.identifier, self.current_mac_type)

                pkd.waiting_start_time = self.env.now

                if self.transmitting_queue.qsize() < self.max_queue_size:
                    self.transmitting_queue.put(pkd)
                else:
                    pass
            else:
                break

    def blocking(self):
        """Original blocking function"""
        if self.enable_blocking:
            if not self.mac_protocol.wait_ack_process_finish:
                flag = False
            else:
                final_indicator = list(self.mac_protocol.wait_ack_process_finish.items())[-1]
                if final_indicator[1] == 0:
                    flag = True
                else:
                    flag = False
        else:
            flag = False
        return flag

    def feed_packet(self):
        """Original feed_packet function"""
        while True:
            if not self.sleep:
                yield self.env.timeout(10)

                if not self.blocking():
                    if not self.transmitting_queue.empty():
                        packet = self.transmitting_queue.get()

                        # Check if packet has deadline attribute
                        has_deadline = hasattr(packet, 'deadline') and hasattr(packet, 'creation_time')
                        
                        # Check expiration for data packets
                        if not has_deadline or self.env.now < packet.creation_time + packet.deadline:
                            if isinstance(packet, DataPacket):
                                # Check retransmission attempts
                                max_retrans = getattr(config, 'MAX_RETRANSMISSION_ATTEMPT', 5)
                                if packet.number_retransmission_attempt[self.identifier] < max_retrans:
                                    has_route, final_packet, enquire = self.routing_protocol.next_hop_selection(packet)

                                    if has_route:
                                        logger.info('At time: %s (us) ---- UAV: %s obtain next hop: %s using %s',
                                                    self.env.now, self.identifier, packet.next_hop_id, 
                                                    self.current_mac_type)
                                        yield self.env.process(self.packet_coming(final_packet))
                                    else:
                                        self.waiting_list.append(packet)
                                        self.remove_from_queue(packet)
                                        if enquire:
                                            yield self.env.process(self.packet_coming(final_packet))
                                else:
                                    logger.info('At time: %s (us) ---- Packet %s dropped (max retransmissions)',
                                                self.env.now, packet.packet_id)
                            else:
                                # Control packet - process directly
                                yield self.env.process(self.packet_coming(packet))
                        else:
                            logger.info('At time: %s (us) ---- Packet %s dropped (expired)',
                                        self.env.now, getattr(packet, 'packet_id', 'Unknown'))
            else:
                break

    def packet_coming(self, pkd):
        """Original packet_coming function with metrics tracking"""
        if not self.sleep:
            arrival_time = self.env.now
            logger.info('At time: %s (us) ---- Packet: %s waiting for UAV: %s buffer (%s)',
                        arrival_time, pkd.packet_id, self.identifier, self.current_mac_type)

            with self.buffer.request() as request:
                yield request

                logger.info('At time: %s (us) ---- Packet: %s in buffer of UAV: %s, wait: %s us (%s)',
                            self.env.now, pkd.packet_id, self.identifier, 
                            self.env.now - arrival_time, self.current_mac_type)

                pkd.number_retransmission_attempt[self.identifier] += 1

                if pkd.number_retransmission_attempt[self.identifier] == 1:
                    pkd.time_transmitted_at_last_hop = self.env.now

                logger.info('At time: %s (us) ---- Retransmission attempts of pkd: %s at UAV: %s is: %s',
                            self.env.now, pkd.packet_id, self.identifier,
                            pkd.number_retransmission_attempt[self.identifier])

                self.mac_process_count += 1
                key = ''.join(['mac_send', str(self.identifier), '_', str(pkd.packet_id)])

                mac_process = self.env.process(self.mac_protocol.mac_send(pkd))
                self.mac_process_dict[key] = mac_process
                self.mac_process_finish[key] = 0

                yield mac_process
                
                # Record transmission attempt after MAC process completes
                # Success is determined by whether ACK was received
                success = self.mac_process_finish.get(key, 0) == 1
                self.record_transmission_attempt(success)
        else:
            pass

    def remove_from_queue(self, data_pkd):
        """Original remove_from_queue function"""
        temp_queue = queue.Queue()
        while not self.transmitting_queue.empty():
            pkd_entry = self.transmitting_queue.get()
            if pkd_entry != data_pkd:
                temp_queue.put(pkd_entry)
        while not temp_queue.empty():
            self.transmitting_queue.put(temp_queue.get())

    def receive(self):
        """Original receive function with metrics tracking"""
        while True:
            if not self.sleep:
                self.update_inbox()
                flag, all_drones_send_to_me, time_span, potential_packet = self.trigger()

                if flag:
                    transmitting_node_list = []
                    for drone in self.simulator.drones:
                        for item in drone.inbox:
                            packet = item[0]
                            insertion_time = item[1]
                            transmitter = item[2]
                            channel_used = item[4]
                            transmitting_time = packet.packet_length / config.BIT_RATE * 1e6
                            interval = [insertion_time, insertion_time + transmitting_time]

                            for interval2 in time_span:
                                if has_intersection(interval, interval2):
                                    transmitting_node_list.append([transmitter, channel_used])

                    transmitting_node_list = [list(x) for x in {tuple(i) for i in transmitting_node_list}]
                    sinr_list = sinr_calculator(self, all_drones_send_to_me, transmitting_node_list)

                    max_sinr = max(sinr_list)
                    if max_sinr >= config.SNR_THRESHOLD:
                        which_one = sinr_list.index(max_sinr)
                        pkd = potential_packet[which_one]

                        # Check if packet has TTL attribute (data packets)
                        if hasattr(pkd, 'get_current_ttl'):
                            if pkd.get_current_ttl() < config.MAX_TTL:
                                sender = all_drones_send_to_me[which_one][0]

                                logger.info('At time: %s (us) ---- Packet %s from UAV: %s received by UAV: %s, sinr: %s (%s)',
                                            self.env.now, pkd.packet_id, sender, self.identifier, max_sinr, 
                                            self.current_mac_type)

                                # Record reception for metrics (only for data packets)
                                if isinstance(pkd, DataPacket):
                                    self.record_packet_reception(pkd)
                                
                                yield self.env.process(self.routing_protocol.packet_reception(pkd, sender))
                            else:
                                logger.info('At time: %s (us) ---- Packet %s dropped (max TTL exceeded)',
                                            self.env.now, pkd.packet_id)
                        else:
                            # Control packet without TTL check
                            sender = all_drones_send_to_me[which_one][0]
                            
                            logger.info('At time: %s (us) ---- Control packet %s from UAV: %s received by UAV: %s (%s)',
                                        self.env.now, getattr(pkd, 'packet_id', 'Unknown'), sender, 
                                        self.identifier, self.current_mac_type)
                            
                            yield self.env.process(self.routing_protocol.packet_reception(pkd, sender))
                    else:
                        pass

                yield self.env.timeout(5)
            else:
                break

    def update_inbox(self):
        """Original update_inbox function"""
        if config.VARIABLE_PAYLOAD_LENGTH:
            max_transmission_time = ((config.AVERAGE_PAYLOAD_LENGTH + config.MAXIMUM_PAYLOAD_VARIATION)
                                     / config.BIT_RATE) * 1e6
        else:
            max_transmission_time = (config.AVERAGE_PAYLOAD_LENGTH / config.BIT_RATE) * 1e6

        for item in self.inbox:
            insertion_time = item[1]
            received = item[3]
            if insertion_time + 2 * max_transmission_time < self.env.now:
                if received:
                    self.inbox.remove(item)

    def trigger(self):
        """Original trigger function"""
        flag = 0
        all_drones_send_to_me = []
        time_span = []
        potential_packet = []

        for item in self.inbox:
            packet = item[0]
            insertion_time = item[1]
            transmitter = item[2]
            processed = item[3]
            channel_used = item[4]
            transmitting_time = packet.packet_length / config.BIT_RATE * 1e6

            if not processed:
                if self.env.now >= insertion_time + transmitting_time:
                    flag = 1
                    all_drones_send_to_me.append([transmitter, channel_used])
                    time_span.append([insertion_time, insertion_time + transmitting_time])
                    potential_packet.append(packet)
                    item[3] = 1
                else:
                    pass
            else:
                pass

        return flag, all_drones_send_to_me, time_span, potential_packet
    
    # ============================================================================
    # UTILITY FUNCTIONS FOR ANALYSIS
    # ============================================================================
    
    def get_performance_comparison(self):
        """
        Returns comparison of TDMA vs CSMA performance.
        Useful for post-simulation analysis.
        """
        comparison = {
            'TDMA': {
                'avg_pdr': np.mean(self.metrics_tdma['pdr']) if self.metrics_tdma['pdr'] else 0,
                'avg_throughput': np.mean(self.metrics_tdma['throughput']) if self.metrics_tdma['throughput'] else 0,
                'avg_delay': np.mean(self.metrics_tdma['delay']) if self.metrics_tdma['delay'] else 0,
                'avg_energy': np.mean(self.metrics_tdma['energy']) if self.metrics_tdma['energy'] else 0,
                'total_collisions': sum(self.metrics_tdma['collisions']),
                'samples': len(self.metrics_tdma['pdr'])
            },
            'CSMA': {
                'avg_pdr': np.mean(self.metrics_csma['pdr']) if self.metrics_csma['pdr'] else 0,
                'avg_throughput': np.mean(self.metrics_csma['throughput']) if self.metrics_csma['throughput'] else 0,
                'avg_delay': np.mean(self.metrics_csma['delay']) if self.metrics_csma['delay'] else 0,
                'avg_energy': np.mean(self.metrics_csma['energy']) if self.metrics_csma['energy'] else 0,
                'total_collisions': sum(self.metrics_csma['collisions']),
                'samples': len(self.metrics_csma['pdr'])
            }
        }
        
        return comparison
    
    def export_metrics_to_csv(self, filename=None):
        """
        Exports collected metrics to CSV for analysis.
        Creates separate files for TDMA and CSMA metrics.
        """
        import csv
        from datetime import datetime
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"drone_{self.identifier}_metrics_{timestamp}"
        
        # Export TDMA metrics
        with open(f"{filename}_TDMA.csv", 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Sample', 'PDR', 'Throughput', 'Delay', 'Energy', 'Collisions', 'Contention'])
            for i in range(len(self.metrics_tdma['pdr'])):
                writer.writerow([
                    i,
                    self.metrics_tdma['pdr'][i],
                    self.metrics_tdma['throughput'][i],
                    self.metrics_tdma['delay'][i],
                    self.metrics_tdma['energy'][i],
                    self.metrics_tdma['collisions'][i],
                    self.metrics_tdma['contention_level'][i] if i < len(self.metrics_tdma['contention_level']) else 0
                ])
        
        # Export CSMA metrics
        with open(f"{filename}_CSMA.csv", 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Sample', 'PDR', 'Throughput', 'Delay', 'Energy', 'Collisions', 'Contention'])
            for i in range(len(self.metrics_csma['pdr'])):
                writer.writerow([
                    i,
                    self.metrics_csma['pdr'][i],
                    self.metrics_csma['throughput'][i],
                    self.metrics_csma['delay'][i],
                    self.metrics_csma['energy'][i],
                    self.metrics_csma['collisions'][i],
                    self.metrics_csma['contention_level'][i] if i < len(self.metrics_csma['contention_level']) else 0
                ])
        
        logger.info('Metrics exported to %s_TDMA.csv and %s_CSMA.csv', filename, filename)
