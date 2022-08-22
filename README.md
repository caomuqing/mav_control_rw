mav_control_rw [![Build Status](https://travis-ci.org/ethz-asl/mav_control_rw.svg?branch=master)](https://travis-ci.org/ethz-asl/mav_control_rw)
======

Control strategies for rotary wing Micro Aerial Vehicles (MAVs) using ROS

Overview
------

This repository contains controllers for rotary wing MAVs. Currently we support the following controllers:
- *mav_linear_mpc* : Linear MPC for MAV trajectory tracking
- *mav_nonlinear_mpc* : Nonlinear MPC for MAV trajectory tracking
- *PID_attitude_control* : low level PID attitude controller 

Moreover, an external disturbance observer based on Kalman Filter is implemented to achieve offset-free tracking. 

If you use any of these controllers within your research, please cite one of the following references

```bibtex
@incollection{kamelmpc2016,
                author      = "Mina Kamel and Thomas Stastny and Kostas Alexis and Roland Siegwart",
                title       = "Model Predictive Control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System",
                editor      = "Anis Koubaa",
                booktitle   = "Robot Operating System (ROS) The Complete Reference, Volume 2",
                publisher   = "Springer",
                year = “2017”,
}
```

```bibtex
@ARTICLE{2016arXiv161109240K,
          author = {{Kamel}, M. and {Burri}, M. and {Siegwart}, R.},
          title = "{Linear vs Nonlinear MPC for Trajectory Tracking Applied to Rotary Wing Micro Aerial Vehicles}",
          journal = {ArXiv e-prints},
          archivePrefix = "arXiv",
          eprint = {1611.09240},
          primaryClass = "cs.RO",
          keywords = {Computer Science - Robotics},
          year = 2016,
          month = nov
}

```


