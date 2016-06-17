#MControl

PX4/NuttX application to activate offboard control mode and control the 
drone internaly.

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
- Add pthreads
- Test on the real drone
- Add higher level commands 
