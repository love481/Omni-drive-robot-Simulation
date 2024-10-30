# Omni-drive-robot-Simulation
Comparative Analysis of NMPC and Fuzzy PID Controllers for Trajectory Tracking in Omni-Drive Robots: Design, Simulation, and Performance Evaluation. The paper is published in International Journal of Fuzzy Systems(Impact factor: 4.3) and can be accessed through following paper [Link](https://link.springer.com/epdf/10.1007/s40815-024-01866-1?sharing_token=UqRV1iw394HkW4Uus6O2PPe4RwlQNchNByi7wbcMAY63Ddx6UAxg0RirbbSdagrpLFM3IDU7k2ClLXEIKxylGmmP2vkoUkUDJSyi9N9MytUAKpoc_gEcJkTQJAzTdkJYOm1Wk_5uNma2AoA6TpDJJi1IgSkAexuFkMK0Iv2qEoI%3D).

## Requirements:
* matlab/simulink
* CoppeliaSim Edu version

## Installation
Run this command on command prompt to clone the repository:

`git clone https://github.com/love481/Omni-drive-robot-Simulation.git`


## Usage
Use the following script to initialize the path of working directory:
```
startup.m
```

Make sure to open the CoppeliaSim simulation environment before running below code.
Use the following scripts for running the simulation:
```
# For tracking using fuzzyPID
fuzzyPID_vrep.m

# For tracking using NMPC
mpc_vrep.m

```

## folder Structure
* `scene` --> Simulation Environment for the robot
* `code/common` --> Contains the common utility classes or functions needed to initialize the remote connection, initializing different PID algorithms, trajectory generators, transformation and so on.
* `code/omni_drive` --> Contains Fuzzy inference system along with scripts for trajectory tracking of omni-drive robot using both type fuzzyPID and NMPC.


#### Visualization of Position and Velocity Tracking for NMPC
![Non-linear Model predictive controller](mpc_trajectory.jpg)

## Citation
If any part of our paper and code is helpful to your work, please generously cite with:
```
@article{panta2024comparative,
  title={Comparative Analysis of NMPC and Fuzzy PID Controllers for Trajectory Tracking in Omni-Drive Robots: Design, Simulation, and Performance Evaluation},
  author={Panta, Love},
  journal={International Journal of Fuzzy Systems},
  pages={1--11},
  year={2024},
  publisher={Springer}
}
```

## Contact Information
Please feel free to contact me if any help needed

Email: *075bei016.love@pcampus.edu.np*

