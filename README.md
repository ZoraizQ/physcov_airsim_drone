# PhysCov - Quantifying Physical Coverage for Autonomous Drones (Microsoft AirSim)
Thorough testing of autonomous systems for UAVs before their deployment is crucial to guarantee safety and mitigate costly failures. Existing software testing coverage metrics lack interpretability and these test adequacy metrics are not effective for complex AI driven autonomous vehicles. We extend PhysCov, a new physical coverage metric for autonomous vehicles, to the domain of UAVs - to measure coverage on a Software-In-The-Loop simulation tool like Microsoft AirSim. This work is an extension of the original coverage metric proposed by Carl Hildebrandt (PhD Computer Science Student, UVA) under the supervision of Dr. Sebastian Elbaum (Professor Computer Science, UVA).

Along with integration for real-time coverage measurements, we test various scenarios in AirSim using a quadrotor equipped with a LIDAR sensor under certain defined survey tasks. Based on our evaluation, we propose that gathering high coverage with PhysCov indicates testing of an adequate number of distinct behaviors for an autonomous flying system, uncovering potential failures such as collisions.

You can read the full project report [here](Report.pdf).

<img src="assets/lidar.jpg" width="50%" height="50%" />

## Dependencies
1. Install [Python](https://www.python.org/downloads/). Preferably v3.8, if you would like to use Open3D for pointcloud visualization during RSR generation.
2. Install the following packages: 
```
pip install numpy
pip install msgpack-rpc-python
pip install -r requirements.txt
```
Some of these initial dependencies were made to install separately given they conflict with other requirements later.

## Setup an AirSim Environment

1. Download any environment from the link below, for example, Blocks.zip. Extract and run the .exe
https://github.com/Microsoft/AirSim/releases


2. Place the settings.json file in Documents/AirSim **OR** run the following command with the executable of your environment:
![sim](assets/sim.jpg)
```
.\Blocks\WindowsNoEditor\Blocks.exe --settings settings.json
```

You can press F1 to view more help information inside the simulation tool. 
Please go over the instructions to use AirSim at https://microsoft.github.io/AirSim/#how-to-use-it

### Hardware Requirements
AirSim is built on Unreal Engine and can be hardware intensive ([FAQs](https://microsoft.github.io/AirSim/faq/)) - the system requires at least 4GB of RAM (8GB recommended) and an integrated/external GPU with atleast 4GB of VRAM. To obtain the best performance without a dedicated GPU, please use the Blocks environment provided by AirSim as this has a minimal hardware footprint.

## Choose a Control Mode
Choose from the following modes currently:

### RECORD
Manual control with recording using an [Xbox controller](https://microsoft.github.io/AirSim/xbox_controller/) connected to your computer. 

Other alternatives including programmatic controls such as pre-defined survey paths using the AirSim Python API, or setting up configuration for a PX4 controller in AirSim which we have not tested. More instructions regarding these procedures can be found at https://microsoft.github.io/AirSim/#how-to-use-it.

### PLAYBACK 
This involves replaying a recorded path (autonomous playback mode) from the last recorded manual session - saved as `recordedpath.npy` (which includes 3D coordinates traversed at captured timepoints). 

**Note:** The recorded path provided inside the repository was recorded over the Blocks.exe environment. To test this mode out, setup that environment and simply run the script with autonomous playback mode set.

### SURVEY
This involves specifying a planar survey path for the drone to traverse starting from its initial position. PhysCov is computed in a separate thread as this executes. *PhysCov computation starts after the drone reaches the first survey box corner.* 
```
boxsize:	The overall size of the square box to survey
stripewidth:	How far apart to drive the swim lanes, this can depend on the type of camera lens, for example.
altitude:	The height to fly the survey.
speed:	The speed of the survey can depend on how fast your camera can snap shots.
```
Extracted from https://microsoft.github.io/AirSim/drone_survey/.


## Start real-time PhysCov generation

Once the installed Airsim environment binary (e.g. Blocks.exe) is running, simply execute the main script with the default parameters:
```python src/physcov_airsim_drone.py```

To view the HELP guide for all configurable parameters (described next section):
```python src/physcov_airsim_drone.py -h```

The script will initiate drone take-off using the pre-defined default parameters and on a separate thread perform RSR generation and PhysCov computations while the drone flies. 

After execution, plots for the runs are also saved in the 'results' folder, including plots for:
- PhysCov over time
- Number of unique collisions over time
- Number of unique collision vectors

## Configurable Parameters
These can be viewed again with the -h or --help flags.
```
usage:
GENERAL: control_mode,
PHYSCOV: t, x, g, R, srad, collision_angle_thresh, \SURVEY: boxsize, stripewidth, altitude, speed
       [-h] [--control_mode CONTROL_MODE] [--t T] [--x X] [--g G] [--R R]
       [--collision_angle_thresh COLLISION_ANGLE_THRESH] [--srad SRAD] [--boxsize BOXSIZE] [--stripewidth STRIPEWIDTH]
       [--altitude ALTITUDE] [--speed SPEED]

optional arguments:
  -h, --help            show this help message and exit
  --control_mode CONTROL_MODE
                        record, playback, survey
  --t T                 timesteps for the simulation (# RSR signatures)
  --x X                 number of equidistant beams / rays
  --g G                 granularity (for beam length rounding) (m)
  --R R                 spherical reachable range radius of the drone (assume same as speed) (m)
  --collision_angle_thresh COLLISION_ANGLE_THRESH
                        collision angle threshold to mark a unique collision
  --srad SRAD           radius of sphere for sphere-casting for lidar truncation (m)
  --boxsize BOXSIZE     size of the box to survey
  --stripewidth STRIPEWIDTH
                        stripe width of survey (in meters)
  --altitude ALTITUDE   altitude of survey (in positive meters)
  --speed SPEED         speed of survey (in meters/second)
```

PhysCov Parameters Defaults:

<img src="assets/formula.png" width="20%" height="20%" />

- Simulation timesteps for recording (based on and equivalent to number of RSR signatures) **(t = 500)**
- Number of beams/rays **(x = 5)**
- Granularity for beam distance rounding **(g = 2)**
- Drone velocity for playback mode **(v = 6 m/s)**
- Max reachable range for one timepoint **(R = 6 m)** 
- Radius of sphere for sphere-casting collision detection of RSR beams **(r = 1m)**

![params](assets/params.png)

## Sample usage:
```
python src\physcov_airsim_drone.py --control_mode survey --t 50 --x 10 --g 15 --R 60
```
This initiates the drone in survey mode (default survey params), with 50 timesteps, 10 beams (RSR-10), granularity of 15m and reachable range of 60m. 
PhysCov updates are also logged live inside AirSim now (blue console text).
![survey-test](assets/survey-test.png)

```
python src\physcov_airsim_drone.py --control_mode playback --x 5 --g 2 --R 6
```
This plays back the saved recorded path present in the repository with RSR-5, granularity of 2 and reachable range of 6m.

## Issues?
**AirSim not responding / Client API connection fails?** - Reset the drone (Backspace), or restart the AirSim environment.
**BufferError?** - If you get a BufferError such as "Existing exports of data: object cannot be re-sized on the console" - simply reset the drone using backspace and re-run the script.

## Collaborators

- Zoraiz Qureshi (zoraizq@virginia.edu) - MS Computer Science Student, University of Virginia
- Shreyes Bhat (shreyes@virginia.edu) - MS Computer Science Student, University of Virginia
- Carl Hildebrandt (hildebrandt.carl@outlook.com) - PhD Computer Science Student, University of Virginia
- Dr. Sebastian Elbaum (selbaum@virginia.edu) - Professor Computer Science, University of Virginia
