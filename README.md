# CARLA vehicle simulation (CARLA v0.9.16)

This guide provides instructions for setting up and running a vehicle simulation in the CARLA simulator to generate synthetic sensor data. The simulation software records sensor information for six cameras and a LiDAR sensor. The sensors are positioned to mimic NuScenes dataset sensor setup. 

*Note: The article "On the Impact of Video Compression on Vision-Based Occupancy
Prediction" is based on commit [d1a6ab8](https://github.com/TO-autonomy/CARLA-vehicle-simulation/tree/d1a6ab86d489e1b27c9eaca6d6c777b5bcb04788). It is strongly recommended to use this version of the pipeline to reproduce results.*

## System Requirements

The CARLA simulation was tested with a high-end workstation. The pipeline should work with lower system specifications, but the stability and effectiveness are unknown. **Software and Hardware requirements need to be fulfilled**!

### Hardware Requirements
- **NVIDIA GPU**: VRAM >= 8GB (NVIDIA drivers need to be installed and working)

### Software Requirements
- **OS**: Ubuntu 22.04 (other versions not tested; Windows and other OS systems do not work)
- **Python Version**: 3.10 / 3.11 (CARLA Python library requirement)

### Tested System Specification
- **Operating System:** Ubuntu 22.04
- **Architecture:** 64-bit
- **Processor:** 12th Gen Intel® Core™ i9-12900K
- **RAM:** 125.51 GB
- **Python Version:** 3.10
- **GPU:** NVIDIA GeForce RTX 4090
- **Storage:** SSD with at least 20 GB of free space

---

## Environment Setup (highly recommended)

We recommend using Conda to manage the Python environment for this project. Conda installation guide can be found here: [Miniconda installer (recommended)](https://www.anaconda.com/docs/getting-started/miniconda/install#linux-2).

After installing Conda create and activate a new environment:

```bash
conda create -n carla-env python=3.11 -y
conda activate carla-env
```

Other environment tools like pyenv and virtualenv should be fine. The key requirement is that the running Python version is 3.10 or 3.11. Once the environment is active, continue with the installation steps below.

---
## Installation

Clone the CARLA vehicle simulation repository:

```bash
git clone https://github.com/TO-autonomy/CARLA-vehicle-simulation.git
cd CARLA-vehicle-simulation
```

Setup repository and install dependencies (including the precompiled **CARLA v0.9.16** simulator):

```bash
chmod +x *.sh
./install.sh
```

---

## Running the Simulation

Start the default simulation scenario:

```bash
./run_simulation.sh
```

This command will:
- Launch the CARLA simulator  
- Run the default data collection scenario  

Sensor data will be saved in:

```
.../CARLA-vehicle-simulation/generated_data
```

---

## Creating a Custom Simulation

### Plan-based Simulation

To create a custom simulation plan, run:

```bash
./make_simulation.sh custom_scenario.toml
```

This will open the simulation planner, where you can define scenario parameters. Your configuration will be saved to `custom_scenario.toml`.  
Then, start the simulation with:

```bash
./run_simulation.sh custom_scenario.toml
```

The simulation will use your custom configuration and generate sensor data for that scenario.

### Recording-based Simulation

To create a simulation based on a recorded driving path, run:

```bash
./make_simulation.sh recording.rec
```

This launches the simulation recorder, allowing you to control an ego vehicle and record the entire simulation scenario (all actors and events will be saved to `recording.rec`). 
Then, run the recording with:

```bash
./run_simulation.sh recording.rec
```

The system will replay your recording and generate sensor data for that scenario.

---

## Enabling Data Post-Processing

Data post-processing is **disabled by default**.  
To enable it:

1. Open `run_simulation.sh` in a text editor.  
2. Uncomment the post-processing lines at the end of the script.

When enabled, raw simulation data is processed and exported into a structured dataset format.

---

## Enabling Data Visualization

Data visualization is **disabled by default**.  
To enable it:

1. Open `run_simulation.sh` in a text editor.  
2. Uncomment the visualization lines at the end of the script.

With visualization enabled, the outputs from the **front cameras** and **LiDAR** are rendered and saved in the specified target folder.

---










