## CARLA vehicle simulation (CARLA v0.9.15)

This guide provides instructions for setting up and running a vehicle simulation in the CARLA simulator to generate synthetic sensor data. The simulation software records sensor information for six cameras and a LiDAR sensor. The sensors are positioned to mimic NuScenes dataset sensor setup. 

*Note: The article "On the Impact of Video Compression on Vision-Based Occupancy
Prediction" is based on commit [d1a6ab8](https://github.com/TO-autonomy/CARLA-vehicle-simulation/tree/d1a6ab86d489e1b27c9eaca6d6c777b5bcb04788). It is strongly recommended to use this version of the pipeline to reproduce results.*

## System Requirements

The CARLA simulation was tested with a high-end workstation. However, the recommended system specification for the CARLA simulator should also work for running the vehicle simulation. Lower system requirements could work in a limited capacity, but the simulation's stability and effectiveness are unknown. **NB! The system was developed for Ubuntu 20.04 and Ubuntu 22.04 version. Windows or other OS systems are expected to cause issues during setup or simulation.**

### Tested System Specification
- **Operating System:** Ubuntu 20.04 (Ubuntu 22.04 works as well)
- **Architecture:** 64-bit
- **Processor:** 12th Gen Intel® Core™ i9-12900K
- **RAM:** 125.51 GB
- **Python Version:** 3.8.10
- **GPU:** NVIDIA GeForce RTX 4090
- **Storage:** SSD with at least 20 GB of free space

### Recommended System Specification
- **Operating System:** Ubuntu 20.04 OR Ubuntu 22.04
- **Python Version:** 3.8
- **Architecture:** 64-bit
- **Processor:** 8th Gen Intel® Core™ i7 or AMD Ryzen™ 7 (8 cores or higher)
- **RAM:** 32 GB
- **Python Version:** 3.8 or higher
- **GPU:** NVIDIA GeForce RTX 3060 or higher with at least 8 GB VRAM
- **Storage:** SSD with at least 20 GB of free space

## Installation

Clone the CARLA vehicle simulation repository and install the necessary dependencies, including a  compiled version of the CARLA simulator (version 0.9.15):

```bash
git clone https://github.com/TO-autonomy/CARLA-vehicle-simulation.git
cd CARLA-vehicle-simulation
sh install.sh
```

## Running the Simulation

To start the vehicle simulation, execute the following command:

```bash
sh run_simulation.sh
```
The script should start the CARLA simulator (if it is installed properly using the install script above) and start the data collection process.

During the simulation run, sensor readings are stored in the following directory:
```
.../CARLA-vehicle-simulation/src/generated_data
```

## Enabling Data Post-Processing

Simulation data post-processing is disabled by default. To enable it:
1. Open the `run_simulation.sh` script in a text editor.
2. Uncomment the relevant lines at the end of the file to activate post-processing.

When data post-processing is enabled, the raw simulation data is processed and exported into a dataset structure. 


