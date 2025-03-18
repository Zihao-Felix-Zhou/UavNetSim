<div align="center">
<img src="https://github.com/Zihao-Felix-Zhou/UavNetSim/blob/master/img/logo.png" width="650px">
</div>

<div align="center">
  <h1>UavNetSim: A Simulation Platform for UAV Networks</h1>

  <img src="https://img.shields.io/badge/Github-%40ZihaoZhouSCUT-blue" height="20">
  <img src="https://img.shields.io/badge/Contribution-Welcome-yellowgreen" height="20">
  <img src="https://img.shields.io/badge/License-MIT-brightgreen" height="20">

  <h3>Make simulation more friendly to novices! </h3>
</div>

This Python-based simulation platform can realistically model various components of the UAV network, including the network layer, MAC layer and physical layer, as well as the UAV mobility model, energy model, etc. In addition, the platform can be easily extended to meet the needs of different users and develop their own protocols.

## Requirements
- Python >= 3.3 
- Simpy >= 4.1.1

## Features
Before you start your simulation journey, we recommend that you read this section first, in which some features of this platform are mentioned so that you can decide if this platform meets your development or research needs.
- Python-based (this simulation platform is developed based on SimPy library in Python);
- More suitable for the development and verification of **routing protocols**, **MAC protocols**, and **motion control algorithms** (e.g., **topology control**, **trajectory optimization**). In the future, we hope to improve the platform to support more kinds of algorithms and protocols at different layers;
- Support **reinforcement learning (RL)** and other AI-based algorithms;
- Easy to extend (1. **modular programming** is used, and users can easily add their own designed modules; 2. different application scenarios are possible, e.g., **flying ad-hoc networks (FANETs)**, **UAV-assisted data collection**, **air-ground integrated network**);
- **Good visulization**, this platform can provide the visualization of **UAV flight trajectory** and **packet forwarding path**, which facilitates intuitive analysis of the behavior of protocols;
- If you are engaged in UAV-assisted wireless communication systems and want to **consider more cross-layer metrics** (e.g., end-to-end delay, packet delivery ratio (PDR), throughput), then this platform is for you

## Project structure
```
.
├── README.md
├── energy
│   └── energy_model.py
├── entities
│   ├── drone.py
│   └── packet.py
├── mac
│   ├── csma_ca.py
│   └── pure_aloha.py
├── mobility
│   ├── gauss_markov_3d.py
│   ├── random_walk_3d.py
│   ├── random_waypoint_3d.py
│   └── start_coords.py
├── phy
│   ├── channel.py
│   ├── large_scale_fading.py
│   └── phy.py
├── routing
│   ├── dsdv
│   │   ├── dsdv.py
│   │   ├── dsdv_packet.py
│   │   └── dsdv_routing_table.py
│   ├── grad
│   │   └── ...
│   ├── greedy
│   │   └── ...
│   ├── opar
│   │   └── ...
│   └── q_routing
│       └── ...
├── simulator
│   ├── metrics.py
│   └── simulator.py
├── topology
│   └── virtual_force
│       ├── vf_motion_control.py
│       ├── vf_neighbor_table.py
│       └── vf_packet.py
├── utils
│   ├── config.py
│   ├── ieee_802_11.py
│   └── util_function.py
├── visualization
│   ├── scatter.py
│   └── visualizer.py
└── main.py
```
The entry point of this project is the ```main.py``` file, we can even run it directly with one click to get a sneak peek, however, we recommend that you first read this section to understand the modular composition of this simulation platform and the corresponding function.

- ```energy```: this module implements the energy model of the drone, including the flying energy consumption and the energy consumption related to communication.
- ```entities```: it includes all the classes that define the behavior and the structure of the key entities in the simulation.
- ```mac```: it includes the implementation of different medium access control (MAC) protocols, e.g., CSMA/CA, ALOHA.
- ```mobility```: it contains different 3-D mobility models of drones, e.g., Gauss-Markov mobility model, random walk, and random waypoint.
- ```phy```: it mainly includes the modeling of wireless channels in the physical layer, and the definition of unicast, broadcast, and multicast.
- ```routing```: it includes the implementation of different routing protocols, e.g., DSDV, GRAd, greedy routing, Q-Learning-based routing, etc.
- ```simulator```: it contains all the classes to handle a simulation and the network performance metrics.
- ```topology```: it includes the implementation of the topology control algorithm for UAV swarm.
- ```utils```: it contains the key configuration parameters and some useful functions.
- ```visualization```: it can provide visualization of the distribution of drones, flight trajectory and the packet forwarding paths.

## Installation and usage
Firstly, download this project:
```
git clone https://github.com/Zihao-Felix-Zhou/UavNetSim.git
```
Run ```main.py``` to start the simulation. 

## Core logic
The following figure shows the main procedure of packet transmissions in *UavNetSim*. "Drone's buffer" is a resource in SimPy whose capacity is one, which means that the drone can send at most one packet at a time. If there are many packets that need to be transmitted, they need to queue for buffer resources according to the time order of arrival to the drone. We can simulate the queuing delay by this mechanism. Besides, we note that there are two other containers: ```transmitting_queue``` and ```waiting_list```. For all the "data packets" and "control packets" generated by the drone itself or received from other drones but need to be further forwarded, the drone will first put them into the ```transmitting_queue```. A function called ```feed_packet``` will periodically read the packet at the head of the ```transmitting_queue``` every very short time, and let it wait for the ```buffer``` resource. It should be noted that the "ACK packet" waits for the buffer resource directly without being put into the ```transmitting_queue```.

After the packet is read, a packet type determination will be performed first. If this packet is a control packet (usually no need to decide the next hop), then it will directly start to wait for the buffer resource. When this packet is a data packet, next hop selection will be executed by the routing protocol, if an appropriate next hop can be found, then this data packet can start waiting for buffer resource, otherwise, this data packet will be put into ```waiting_list``` (mainly in reactive routing protocol now). Once the drone has the relevant routing information, it will take this data packet from "waiting_list" and add it back to ```transmitting_queue```.

When the packet gets the buffer resource, MAC protocol will be performed to contend for the wireless channel. When the packet is successfully received by the next hop, packet type determination needs to be performed. For example, if the received packet is a data packet, an ACK packet is needed to reply after an SIFS time. In addition, if the receiver is the destination of the incoming data packet, some metrics will be recorded (PDR, end-to-end delay, etc.), otherwise, it means that this data packet needs to be further relayed so it will be put into the ```transmitting_queue``` of the receiver drone.

<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/transmitting_procedure.png" width="800px">
</div>

## Module overview
### Routing protocol
Packet routing plays an important role in UAV networks, which enables cooperation among different UAV nodes. In this project, **Greedy routing**, **Gradient routing (GRAd)**, **Destination-Sequenced Distance Vector routing (DSDV)**, and some **RL-based routing protocols** have been implemented. The following figure illustrates the basic procedure of packet routing. More detailed information can be found in the corresponding papers [1]-[5].

<div align="center">
<img src="https://github.com/Zihao-Felix-Zhou/UavNetSim/blob/master/img/routing.png" width="700px">
</div>

### Media access control (MAC) protocol
In this project, **basic Carrier-sense multiple access with collision avoidance (CSMA/CA)** and **Pure aloha** have been implemented. I will give a brief overview of the version implemented in this project, and focus on how signal interference and collision are implemented in this project. The following picture shows an example of packet transmission when the basic CSMA/CA (without RTS/CTS) protocol is adopted. When a drone wants to transmit a packet:

1. it first needs to wait until the channel is idle
2. when the channel is idle, the drone starts a timer and waits for ```DIFS+backoff``` periods of time, where the length of backoff is related to the number of re-transmissions
3. if the entire decrement of the timer to 0 is not interrupted, then the drone can occupy the channel and start sending the packet
4. if the countdown is interrupted, it means that the drone loses the game. The drone then freezes the timer and waits for the channel to be idle again before re-starting its timer

<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/csmaca.png" width="800px">
</div>

The following figure demonstrates the packet transmission flow when pure aloha is adopted. When a drone installed a pure aloha protocol wants to transmit a packet:

1. it just sends it, without listening to the channel and random backoff
2. after sending the packet, the node starts to wait for the ACK packet
3. if it receives ACK in time, the ```mac_send``` process will finish
4. if not, the node will wait a random amount of time, according to the number of re-transmission attempts, and then send the packet again

<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/pure_aloha.png" width="800px">
</div>

From the above illustration, we can see that, it is not only two drones sending packets simultaneously that cause packet collisions. If there is an overlap in the transmission time of two data packets, it also indicates that a collision occurs. So in our project, each drone checks its inbox every very short interval and has several important things to do (as shown in the following figure):

1. delete the packet records in its inbox whose distance from the current time is greater than twice the maximum packet transmission delay. This reduces computational overhead because these packets are guaranteed to have already been processed and will not interfere with packets that have not yet been processed
2. check the packet records in the inbox to see which packet has been transmitted in its entirety
3. if there is such a record, then find other packets that overlap with this packet in transmission time in the inbox records of all drones, and use them to calculate SINR.

<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/reception_logic.png" width="800px">
</div>

### Mobility model
The mobility model is one of the most important mudules to show the characteristics of a UAV network more realistically. In this project, **Gauss-Markov 3D mobility model**, **Random Walk 3D mobility model**, and **Random Waypoint 3D mobility model** have been implemented. Specifically, since it is quite difficult to achieve continuous movement of drones in discrete time simulation, we set a ```position_update_interval``` to update the positions of drones periodically, that is, it is assumed that the drone moves continuously within this time interval. If the time interval ```position_update_interval``` is smaller, the simulation accuracy will be higher, but the corresponding simulation time will be longer. Thus, there will be a trade-off. Besides, the time interval that the drone updates its direction can also be set manually. The trajectories of a single drone within 100 seconds of the simulation under the two mobility models are shown as follows:

<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/mobility_model.png" width="800px">
</div>

### Energy model
The energy model of our platform is based on the work of Y. Zeng, et al. The figure below shows the power required for different drone flying speeds. The energy consumption is equal to the power multiplied by the flight time at this speed.
<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/energy_model.png" width="400px">
</div>

### Visualization


## Performance evaluation
Our "FlyNet" platform supports the evaluation of several performance metrics, as follows:

- **Packet Delivery Ratio (PDR)**: PDR is the ratio of the total number of received data packets successfully at all destination drones over the total number of data packets generated by all source drones. It should be noted that PDR excludes redundant data packets. PDR can reflect the reliability of the routing protocol.
- **Average End-to-End Delay (E2E Delay)**: E2E delay is the average time delay for data packets to reach from the source drone to the destination drone. Typically, delay in packet transmission involves "queuing delay", "access delay", "transmission delay", "propagation delay (So small as to be negligible)" and "processing delay".
- **Normalized Routing Load (NRL)**: NRL is the ratio of all routing control packets sent by all drones to the number of received data packets at the destination drones.
- **Average Throughput**: In our platform, the calculation of throughput is: whenever the destination receives a packet, the length of the packet is divided by the end-to-end delay of the packet (because E2E delay involves the re-transmissions of this data packet)
- **Hop Count**: Hop count is the number of router output ports through which the packet should pass.

## Design your own protocol
Our simulation platform can be expanded based on your research needs, including designing your own mobility model of drones (in ```mobility``` folder), mac protocol (in ```mac```folder), routing protocol (in ```routing```folder), and so on. Next, we take routing protocols as an example to introduce how users can design their own algorithms.

 * Create a new package under the ```routing``` folder (Don't forget to add ```__init__.py```)
 * The main program of the routing protocol must contain the function: ```def next_hop_selection(self, packet)``` and ```def packet_reception(self, packet, src_drone_id)```
 * After confirming that the code logic is correct, you can import the module you designed in ```drone.py``` and install the routing module on the drone:
   ```python
   from routing.dsdv.dsdv import Dsdv  # import your module
   ...
   class Drone:
     def __init__(self, env, node_id, coords, speed, inbox, simulator):
       ...
       self.routing_protocol = Dsdv(self.simulator, self)  # install
       ...
   ```

## Reference
[1] C. Perkins and P. Bhagwat, "[Highly dynamic destination-sequenced distance-vector routing (DSDV) for mobile computers](https://dl.acm.org/doi/abs/10.1145/190809.190336)," in *ACM SIGCOMM Computer Communication Review*, vol. 24, no. 4, pp. 234-244, 1994.  
[2] R. Poor, "Gradient routing in ad hoc networks", 2000, [www.media.mit.edu/pia/Research/ESP/texts/poorieeepaper.pdf](www.media.mit.edu/pia/Research/ESP/texts/poorieeepaper.pdf)  
[3] J. Boyan and M. Littman, "[Packet routing in dynamically changing networks: A reinforcement learning approach](https://proceedings.neurips.cc/paper/1993/hash/4ea06fbc83cdd0a06020c35d50e1e89a-Abstract.html)" in *Advances in Neural Information Processing Systems*, vol. 6, 1993.  
[4] W. S. Jung, J. Yim and Y. B. Ko, "[QGeo: Q-learning-based geographic ad hoc routing protocol for unmanned robotic networks](https://ieeexplore.ieee.org/abstract/document/7829268/)," in *IEEE Communications Letters*, vol. 21, no. 10, pp. 2258-2261, 2017.  
[5] M. Gharib, F. Afghah and E. Bentley, "[Opar: Optimized predictive and adaptive routing for cooperative uav networks](https://ieeexplore.ieee.org/abstract/document/9484489)," in *IEEE INFOCOM 2021-IEEE Conference on Computer Communications Workshops (INFOCOM WKSHPS)*, pp. 1-6, 2021.  
[6] A. Colvin, "[CSMA with collision avoidance](cn.overleaf.com/project/678e52bd44cc7c6c70e39d90)," *Computer Communications*, vol. 6, no. 5, pp. 227-235, 1983.  
[7] N. Abramson, "[The ALOHA system: Another alternative for computer communications](n.overleaf.com/project/678e52bd44cc7c6c70e39d90)," in *Proceedings of the November 17-19, 1970, Fall Joint Computer Conference*, pp. 281-285, 1970.

## Contributing
Issues and pull requests are warmly welcome! 

## Show your support
Give a ⭐ if this project helped you!

## License
This project is MIT-licensed.
