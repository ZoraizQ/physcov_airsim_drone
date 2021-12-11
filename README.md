# physcov_airsim_drone
Scaling PhysCov to 3D for drone physical coverage measurement, with real-time computation using LIDAR data from AirSim quadrotor drone.

<img src="assets/lidar.jpg" width="50%" height="50%" />

## Dependencies
```
pip install numpy
pip install msgpack-rpc-python
pip install -r requirements.txt
```

## Run AirSim

1. Download any environment from the link below, for example, Blocks.zip. Extract and run the .exe
https://github.com/Microsoft/AirSim/releases

Place the settings.json file in Documents/AirSim or run the following command with the executable of your environment.

![sim](assets/sim.jpg)
```
.\Blocks\WindowsNoEditor\Blocks.exe --settings settings.json
```
## Start PhysCov generation

Once airsim is running, run drone_lidar.py as follows
```python drone_lidar.py```

## Choose mode
Choose from playing a recorded path replay (autonomous playback mode) or manual control (with Xbox controller)
If the drone isn't controllable, press backspace to reset and you should be able to control it now with an Xbox Controller.

To replay the recorded path autonomously, execute Blocks.exe and run the script.

### if you get a BufferError: Existing exports of data: object cannot be re-sized on the console, reset the drone and re run drone_lidar.py


## note on hardware requirements: AirSim runs using unreal engine and can be hardware intensive, the system requires at least 8GB of RAM and a graphics card with at least 4GB of VRAM. To obtain the best results without a dedicated GPU, please use the Blocks environment provided by airsim as this has a minimal hardware footprint.
