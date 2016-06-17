#MControl

PX4/NuttX application to activate offboard control mode and control the 
drone internaly.

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

You can mode the start() function to change the required setpoints.

# TODO 
- add pthreads

