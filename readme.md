# Online Flocking Control of UAVs with Mean-Field Approximation [ICRA 2021]

This repository contains the code for the ICRA-2021 publication titled "Online Flocking Control of UAVs with Mean-Field Approximation". 

| ![Cover Image](https://github.com/malintha/mean_field_flocking/blob/master/cover.gif?raw=true) |
|:--:| 
| *A swarm of 8 quadrotors demostrating flocking behavior.* |

**Install dependencies**

Please install [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), [Armadillo](https://www.uio.no/studier/emner/matnat/fys/FYS4411/v13/guides/installing-armadillo/) and [GNU Science Library (GSL)](https://www.gnu.org/software/gsl/) before you continue.


**Building the simulator**

This uses a modified version of the Mavswarm simulator. Check https://github.com/Malintha/multi_uav_simulator for the latest version.

Clone the `mean_field_flocking` repository to your catkin workspace. i.e: (~/catkin_ws/src/). 
    
    git clone  https://github.com/malintha/mean_field_flocking


Use catkin build to build the packages as below.
    
    catkin build mrf_dynamics

Run the simulation by launching the `mrf_dynamics/launch/mrf.launch` file. This will create open up a Rviz window and simulate 8 Crazyflie quadrotors in flocking motion.

    source devel/setup.bash
    roslaunch mrf_dynamics mrf.launch

Consider citiing our work if you find this code helpful for your publications.

      @INPROCEEDINGS{9560899,
      author={Fernando, Malintha},
      booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)}, 
      title={Online Flocking Control of UAVs with Mean-Field Approximation}, 
      year={2021},
      volume={},
      number={},
      pages={8977-8983},
      doi={10.1109/ICRA48506.2021.9560899}}
