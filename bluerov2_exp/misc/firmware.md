# Build Firmware from Source

```
./waf configure --board Pixhawk1
./waf -j7 sub
```
The ```ardusub.apj``` file should be under ```~/home/<user>/ardupilot/build/Pixhawk1/bin```; you can then flash this to the onboard controller.