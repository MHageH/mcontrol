#MControl

PX4/NuttX application to activate offboard control mode and control the 
drone internaly.

## Recent mods
- Add pthreads
- Reduce memory consumption
- Changed architecture to daemon application
- Added higher level commands 

- Tested on the real drone

## Capabilities
- Ability to read from and write to drivers and other applications through uORB interprocesses  
communication layer
- Fully multithreaded
- MAVLink protocol independant
- Works internally on the pixhawk without any use of external calculator
- High level functions

## Requirements 
- summon-arm-toolchain or PX4 toolchain 
- PX4 Firmware
- Simulation Dependencies

## Setup
To setup all the requirements, follow this [guide](https://github.com/MHageH/c_uart_interface).

First download the PX4 firmware :
```
git clone https://github.com/PX4/Firmware.git
cd Firmware;
git submodule update --init --recursive
```

Place the content of this git in Firmware/src/examples/mcontrol

Then add this line to Firmware/cmake/configs/posix_sitl_default.cmake
```
examples/mcontrol
```

Go to main repo (Firmware) then :
```
make posix_sitl_default jmavsim
```

The application should compile with the firmware.
Once you have the Nsh access, arm and wait for GPS then takeoff :
```
commander arm
commander takeoff
```
You can now activate the mode by :
```
mcontrol
```

The virtual drone should take position now.

You can mod the start() function to change the required setpoints.

# TODO 
- Add initial position
- Give back control to auto mode on sequence completion
