using Revise

include("./src/WooferSim.jl")
using MuJoCo

mj_activate(ENV["MUJOCO_KEY_PATH"])

using .WooferSim
WooferSim.simulate()
