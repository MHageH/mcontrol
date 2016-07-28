#MControl

PX4/NuttX application to activate offboard control mode and control the 
drone internaly.

## Recent mods
- Added pthreads
- Reduce memory consumption
- Changed architecture to daemon application
- Added higher level commands 
- Added initial position acquisation
- Added get_local_position function
- Give back control to auto mode on sequence completion

- Tested on the real drone (needs real flight testing)

## Capabilities
- Ability to read from and write to drivers and other applications through uORB interprocesses  
communication layer
- Fully multithreaded
- MAVLink protocol independent
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

Then add this line to Firmware/cmake/configs/nuttx_px4fmu-v2_default.cmake
```
examples/mcontrol
```

Go to main repo (Firmware) then :
```
make -j4 px4fmu-v2_default
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

The drone should take position in theory, needs real flight testing.

You can mod the MControl::start() function to change the required setpoints.

Note : This program can be added to simulation, but it won't work (will cause  
the application to crash)

# TODO 
- Add automatic arm/disarm

## Notes
- Disabling the -Werror=double-promotion flag in cmake/common/px4_base.cmake file might be necessary for now