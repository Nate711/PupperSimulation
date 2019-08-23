# WooferJuliaSim

## Overview
A MuJoCo simulator that simulates Woofer QP Balancer and MPC controllers. Ported the simulation over from the Python version (https://github.com/Nate711/QPDoggo).

## Install
1. Acquire a license for MuJoCo at http://mujoco.org/. You can get a free trial of the professional license for a month or get a free year with a student account.
2. Save the license ```mjkey.txt``` somewhere and set the environment variable ```MUJOCO_KEY_PATH``` to that location. One way to set the environment variable is through your bash profile. On a mac this is done by adding the line
```
export MUJOCO_KEY_PATH=[YOUR PATH]/mjkey.txt
```
to your ~/.bash_profile.
3. Add MuJoCo.jl by following the instructions on the [github repo](https://github.com/klowrey/MuJoCo.jl).
4. Add the packages listed below as well with the Julia package manger: Enter Julia REPL. Enter the package manager by typing ']'. Then run
```julia
add [package name]
```

- GLFW
- StaticArrays
- PyCall
- Rotations
- Parameters
- LinearAlgebra
- Plots
- ForwardDiff
- OSQP
- ControlSystems

## Run
1. Enter the Julia REPL in the PupperJuliaSim directory.
2. Run
```julia
include("main.jl")
```
3. The MuJoCo simulator should then pop up in a new window with various interactive options.
- Click and drag with the left mouse button to control the camera view, and with the right mouse button to pan.
- To perturb the robot, double click on the body you want to perturb, then hold Control and click and drag with the mouse. Clicking and dragging with the left mouse button will apply a rotational torque while clicking and dragging with the right button will apply a translational force.
- Press space to play or pause the simulation.
