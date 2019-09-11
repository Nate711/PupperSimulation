using Rotations
using LinearAlgebra
using StaticArrays

@with_kw struct StanceParams
    # Time constant in [sec] for the feet in stance to move towards the reference z height
    ztimeconstant = 1.0
end

function skiincrement(vref::SVector{3, Float64}, wzref::Float64, zref::Float64, zmeas::Float64, dt::Float64, p::StanceParams)
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

    Δp = vref * dt + SVector(0, 0, 1 / p.ztimeconstant * (zref - zmeas)) * dt
    ΔR = RotZ(wzref * dt)
    return (Δp, ΔR)
end
