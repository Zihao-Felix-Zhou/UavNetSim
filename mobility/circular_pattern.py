"""
Circular Pattern Mobility Model for UAV Simulation

This mobility model makes drones fly in circular patterns.
Used as an alternative to random movement patterns.

Author: Enhanced UAV Simulator
Created: 2025/1/13
"""

import math
import numpy as np


class CircularPattern:
    """
    Implements circular movement pattern for drones.
    
    The drone flies in a horizontal circle at constant altitude,
    with configurable radius and angular velocity.
    """
    
    def __init__(self, drone):
        """
        Initialize circular pattern mobility model.
        
        Parameters:
            drone: Reference to the drone object
        """
        self.drone = drone
        self.center = None  # Center of circular path [x, y, z]
        self.radius = 50    # Radius of circle in meters
        self.angular_velocity = 0.1  # Radians per second
        self.current_angle = 0  # Current angle in radians
        self.initialized = False
        
    def set_center(self, center):
        """Set the center point of circular path"""
        self.center = center.copy()
    
    def set_radius(self, radius):
        """Set the radius of circular path"""
        self.radius = radius
    
    def initialize_circular_motion(self):
        """
        Initialize circular motion parameters based on current drone position.
        Calculates the starting angle.
        """
        if self.center is None:
            self.center = self.drone.coords.copy()
        
        # Calculate initial angle based on current position relative to center
        dx = self.drone.coords[0] - self.center[0]
        dy = self.drone.coords[1] - self.center[1]
        
        # If drone is at center, start at angle 0
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            self.current_angle = 0
            # Move drone to starting position on circle
            self.drone.coords[0] = self.center[0] + self.radius
            self.drone.coords[1] = self.center[1]
        else:
            self.current_angle = math.atan2(dy, dx)
        
        self.initialized = True
        self.update_velocity()
    
    def update_velocity(self):
        """
        Updates drone velocity to maintain circular motion.
        Velocity is tangent to the circle at current position.
        """
        # Tangent direction (perpendicular to radius)
        tangent_angle = self.current_angle + math.pi / 2
        
        # Calculate velocity components
        speed = self.drone.speed
        self.drone.velocity[0] = speed * math.cos(tangent_angle)
        self.drone.velocity[1] = speed * math.sin(tangent_angle)
        self.drone.velocity[2] = 0  # Maintain constant altitude
        
        # Update direction and pitch
        self.drone.direction = tangent_angle
        self.drone.pitch = 0  # Horizontal flight
    
    def update_position(self, time_step):
        """
        Update drone position for circular motion.
        
        Parameters:
            time_step: Time elapsed in seconds
        """
        if not self.initialized:
            self.initialize_circular_motion()
        
        # Calculate angular displacement
        delta_angle = (self.drone.speed / self.radius) * time_step
        self.current_angle += delta_angle
        
        # Normalize angle to [0, 2π]
        self.current_angle = self.current_angle % (2 * math.pi)
        
        # Calculate new position on circle
        self.drone.coords[0] = self.center[0] + self.radius * math.cos(self.current_angle)
        self.drone.coords[1] = self.center[1] + self.radius * math.sin(self.current_angle)
        # Z coordinate (altitude) remains constant
        
        # Update velocity for next iteration
        self.update_velocity()
    
    def get_next_position(self, current_time, time_interval):
        """
        Calculate next position based on circular motion.
        
        Parameters:
            current_time: Current simulation time
            time_interval: Time step in microseconds
            
        Returns:
            tuple: (new_x, new_y, new_z) coordinates
        """
        time_step = time_interval / 1e6  # Convert to seconds
        self.update_position(time_step)
        
        return (self.drone.coords[0], self.drone.coords[1], self.drone.coords[2])


