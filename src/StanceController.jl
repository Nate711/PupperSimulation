using Rotations
using LinearAlgebra
using StaticArrays

include("Types.jl")

function skiincrement(vref::SVector{3, Float64}, wzref::Float64, zref::Float64, zmeas::Float64, p::StanceParams, cparams::ControllerParams)
    #=
    Given a desired positioning targets, find the discrete change in position and 
    orientation to the ski for the given timestep duration.
    
    vref: desired body velocity
    wzref: desired body z rotation rate
    zref: desired body x height
    zmeas: actual body z height
    dt: closed loop dt
    p: stance controller parameters
    =#

    Δp = vref * cparams.dt + SVector(0, 0, 1 / p.ztimeconstant * (zref - zmeas)) * cparams.dt
    ΔR = RotZ(wzref * cparams.dt)
    return (Δp, ΔR)
end
