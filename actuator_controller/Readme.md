# Actuator Controller

## Description
- Low-level interface to Faulhaber 2232BX4 motors (integrated motion controllers)
- Set-up of serial communication for all drive units
- Publishing drive information (position / velocities / status) to ROS framework

## Prerequisite (Minimum requirements)
- BeagleBoneBlack with ROS environment and customised header for interfacing with hardware components
- Limit sensors / switches connected to the GPIO interfaces
- Faulhaber Motion controllers (integrated to motors) connected to BeagleBoneBlack via serial interfaces (RS232). Each motors uses a dedicated
hardware interface (/dev/tty00, /dev/tty01, ...). More than 4 devices require additional interfaces, e.g., implemented
by SPI modules. In combination with a kernel module, this enables to extend the number of physical serial interfaces provided by the BeagleBoneBlack
- Faulhaber Motion controllers and motors powered up (required for successful communication)

## Usage

To start communication with all drive units and to enable communication and control via ROS framework

	roslaunch actuator_controller actuator_controller.launch
