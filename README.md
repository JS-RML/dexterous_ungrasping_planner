# Dexterous ungrasping planner
## 1. Overview
This repository contains the MATLAB implementation of dexterous ungrasping planner, which generates the optimal solutions for **Dexterous Ungrasping** (a robotic manipulation technique of securely transfering an object from the gripper to the environment). The planner framework is based on a sampling-based searching algorithm, RRT*, and customized with our contact-mode-based searching function and cost function to explore the configuration space using the predefined motion primitives. With the given information of geometries and friction coefficient of contacts in the object-gripper-environment system, the planner returns a sequence of motion primitives from initial to goal configurations with a secure and collision-free manner that is executable by a real robot platform.  
**Note**: This MATLAB implementation is developed based on [rrt_toolbox](https://github.com/olzhas/rrt_toolbox) built by Olzhas Adiyatov and Atakan Varol.

For details about the implementation of dexterous ungrasping on the UR10 robot arm platform, please refer to [Dexterous Ungrasping](https://github.com/HKUST-RML/shallow_depth_insertion).

**Published Article**

- C. H. Kim and J. Seo, "[Dexerous Ungrasping: Methods and Designs for Secure Placement and insertion through Dexterous Manpulation]()," submitted to *IEEE Transactions on Robotics* (Under review). 

- C. H. Kim and J. Seo, "[Shallow-Depth Insertion: Peg in Shallow Hole Through Robotic In-Hand Manipulation](https://ieeexplore.ieee.org/document/8598749)," in *IEEE Robotics and Automation Letters*, vol. 4, no. 2, pp. 383-390, April 2019.

    *If you use shallow-depth insertion for your application or research, please star this repo and cite our related paper.* [(BibTeX)](files/BibTeX.txt)

**Contributers**: [Chung Hee Kim](https://sites.google.com/view/chjohnkim/home), Ka Hei Mak, and [Jungwon Seo](http://junseo.people.ust.hk/) 

## Maintenance
For any technical issues, please contact John Kim [chkimaa@connect.ust.hk]() and Ka Hei Mak [khmakac@connect.ust.hk]().
