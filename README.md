# Dexterous ungrasping planner
## 1. Overview
This repository contains the MATLAB implementation of dexterous ungrasping planner, which generates the optimal solutions for **Dexterous Ungrasping** (a novel robotic manipulation technique of securely transfering an object from the gripper to the environment). The planner framework is based on a sampling-based searching algorithm, RRT*, and customized with our contact-mode-based searching function and cost function to explore the configuration space using the predefined motion primitives. Given the information of geometries and friction coefficient of contacts in the object-gripper-environment system, the planner returns a sequence of motion primitives from initial to goal configurations with a secure and collision-free manner that is executable by a real robot platform.  

For details about the implementation of dexterous ungrasping on the UR10 robot arm platform, please refer to the package [shallow_depth_insertion](https://github.com/HKUST-RML/shallow_depth_insertion).

**Published Article**

- C. H. Kim and J. Seo, "[Dexerous Ungrasping: Methods and Designs for Secure Placement and insertion through Dexterous Manpulation]()," submitted to *IEEE Transactions on Robotics* (Under review). 

- C. H. Kim and J. Seo, "[Shallow-Depth Insertion: Peg in Shallow Hole Through Robotic In-Hand Manipulation](https://ieeexplore.ieee.org/document/8598749)," in *IEEE Robotics and Automation Letters*, vol. 4, no. 2, pp. 383-390, April 2019.

    *If you use shallow-depth insertion for your application or research, please star this repo and cite our related paper.* [(BibTeX)](files/BibTeX.txt)

**Contributers**: [Chung Hee Kim](https://sites.google.com/view/chjohnkim/home), [Ka Hei Mak](https://github.com/ansonmak), and [Jungwon Seo](http://junseo.people.ust.hk/) 

## 2. Run the planner  
Before running the planner, please ensure the paths in the MATLAB environment are set correctly since this repository contains seperate sets of scripts in `examples/`. For example, if you are running the scripts in `dexterous_ungrasping_planner/`, remove the folder `examples/` and its subfolders from the MATLAB Path to prevent confliction between the sets of scripts.

To start the planner, run:
```
main_DUSimple3D_rrt_star.m
```
### 2.1 Changing planning parameters
The parameters of the planner can be changed through the following .m files:
- **main_DUSimple3D_rrt_star.m** (basic parameters)
   - ***start_point***: Initial configuration
   - ***goal_point***: Goal configuration
   - ***max_iter***: Number of iterations of running the planner
- **configure_DUSimple3D.m** (parameters of RRT* and customized functions)
   - ***delta_goal_point***: Radius for searching the goal point
   - ***delta_near***: Radius for searching the neighboring nodes
   - ***max_step***: Maximum position change when adding a new node to the tree
   - ***mode_change_weight***: Weight for penlizing change of contact mode
   - ***goal_bias***: Probability of sampling points at the goal configuration
- **is_forceclosure.m** (coefficient of friction of contacts)
   - ***phi_G***: Friction angle at **G**
   - ***phi_A***: Friction angle at **A**
   - ***phi_B***: Friction angle at **B**
   - **Note**: *Friction angle*: the arc-tangent of the coefficient of friction (in degree)
- **is_thumb_collision.m** (geometries of thumb modeled as a trapezoid)
   - ***thumb_tip_thickness***: Thickness of the gripper's thumbtip
   - ***thumb_length***: Length of the thumb
   - ***thumb_base_thickness***: Thickness of the thumb's base (including the linkage of the gripper)
   - ***d_FT***: Length difference between the finger and thumb
   - **Note**: the above parameters are divided by the object's length
- **is_finger_collision.m** (geometries of finger modeled as a trapezoid)
   - ***finger_tip_thickness***: Thickness of the gripper's fingertip
   - ***finger_length***: Length of the finger
   - ***finger_base_thickness***: Thickness of the finger's base (including the linkage of the gripper)
   - ***corner_x***: Horizontal poisiton of the obstacle above the finger relative to **G**
   - ***corner_y***: Vertical poisiton of the obstacle above the finger relative to **G**
   - **Note**: the above parameters are divided by the object's length

### 2.2 Generating the regions of C_free and C_obs
The regions of **C_free** and **C_obs** plotted in the result are generated in advance before running the planner. The generated regions are saved as .mat files in `regions/` and being loaded into the program before plotting the result. Some regions based on different finger and thumb settings are provided in this repository.

To generate a new set of regions with the changed parameters and settings in **is_thumb_collision.m**, **is_finger_collision.m**, and **save_map.m**, run:
```
save_map.m
```
After the program is completed, a figure plotting the regions will be shown:
<p align = "center">
<img src="files/Cspace.jpg" height="400">
</p>

Make sure to save the variables: ***P***, ***T_OBS***, and ***F_OBS*** as .mat files in `regions/`. To load the new regions for plotting the firgure, rename the directories correspondingly in `function plot(this)` of `DUSimple3D.m`.

## 3. About the planner
### 3.1 C-space
The configuration space (C-space) is represented by three nondimentionalized parameters in the object-gripper-environment system shown below:
- ***θ*** : Angle between the object and the target surface
- ***ψ*** : Angle between the object and the surface of the thumb
- ***δ_A*** : Ratio of the object enclosed by the gripper over the object length (*δ_A = d_A / ℓ_obj*)
<p align = "center">
<img src="files/Cspace_description.png" height="350">
</p>

where the dimensionless representation of digit asymmetry is defined as: *α = (F_x – T_x) / ℓ_obj* .

The figure in **2.2** shows the C-space represented by **[θ, ψ, δ_A]** with **C_free**: the search space of the planner and **C_obs**: the obstacle space.

### 3.2 Motion primitives
Three motion primitives predefined to navigate in the C-space are as follows:
<p align = "center">
<img src="files/MotionPrimitives.png" height="100">
</p>

The planner is designed to navigate with these motion primitives and return an optimal solution in the C-space from the initial to goal configurations predefined by the user. 

For more details about the C-space, planner framework, and motion primitives, please see our published article, [Dexerous Ungrasping: Methods and Designs for Secure Placement and insertion through Dexterous Manpulation]()

## 4. Planning examples
Several examples under different planning scenarios, for example, gripper configuration or additional obstacle in the environment, are provided in this repository. Details of these planning examples can be found in `examples/README.md`.

## 6. License and original copyright
The source code is released under MIT license. 
**Note**: The source code of this MATLAB implementation is based on [rrt_toolbox](https://github.com/olzhas/rrt_toolbox) built by Olzhas Adiyatov and Atakan Varol licensed under the BSD 3-Clause "New" or "Revised" License. The original copyright notice can be found in `COPYRIGHT.txt`.

## 5. Maintenance
For any technical issues, please contact John Kim [chkimaa@connect.ust.hk]() and Ka Hei Mak [khmakac@connect.ust.hk]().
