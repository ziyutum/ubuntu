# Franka Emika Panda Robot Setup

This is a user manual for the connection of Franka Emika Panda Robot with workstation PC.

## Prerequests

Tested on the following operating systems:
- Windows: 10/11
- Linux System: ubuntu 20.04 with the kernel version 5.9.1-rt20

## Setup

### Equipment overview

![See figure](Robot_basic_Setup.png)

### Basic setup of Hardware

- between the controller and the net supply is situated an emergency stop device, which in cases of emergency safely removes the supply from Panda.

- the 3-way external enabling device is connected at the base of the Arm (connector X4). Half pressing it will activate Panda (Attention-always step out of the safe area first) and programs can be started via Desk.

- The external activation device is connected at the base of the Arm (socket X3), in order to consciously authorize movements of the Arm from outside
the Safety Area.

- the Arm is connected via a connection cable to the Control

- if you wish to program Panda via FCI, the shopfloor network ethernet interface on the front side of the Control should be used.

- switch on the robot and wait until the light turns blue from yellow. During the time, user should stay away from the robot.

### Basic setup of Software

#### connect interface device with robot

- to open the initial configuration interface, an interface device must be connected via Ethernet cable to the X5 connector on the base of the Arm (see in chapter: Mounting & Installation).

- switch off the WLAN of the interface device for windows system

- the interface device must obtain the IP address automatically via DHCP.

- once Panda has been switched on, the interface device will automatically be assigned an IP address.

- then the URL “robot.franka.de” can be entered and opened in a web browser.

- if the URL above cannot be opened, try to configure the robot and franka control interface (FCI) with the following IP address and Netmask manually:

```
Robot:
IP address: 192.168.0.1
Netmask: 255.255.255.0  

```
```
FCI:
IP address: 192.168.10.2
Netmask: 255.255.255.0  
```

#### connect control with Workstation PC
- connect the workstation’s LAN port to the robot’s control unit
- Desk can be accessed via https://192.168.0.1 or https://192.168.10.2



## Enter FCI
- after basic setup is done, the FCI can be entered with the following user data:
```
Username: franka
Password: franka123 
```
- activate the joints
- switch on the robot under robot status
- run the task "Joint Motion" or any arbitrary tasks for testing

## Usage 

tbd



## Reference

- this README.md file is written based on the User manual of https://wwwpub.zih.tu-dresden.de/~s5990957/Franka-Panda-UserGuide-eng.pdf (Page 36, 106)


## Authors and acknowledgment
Xuezhou Hou, Jingyun Zhao TUM AIS

