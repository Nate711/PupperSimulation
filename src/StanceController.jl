using Rotations
using LinearAlgebra
using StaticArrays

include("Types.jl")

function skiincrement(zmeas::Float64, stanceparams::StanceParams, mvref::MovementReference, gaitparams::GaitParams)
    #=
    Given a desired positioning targets, find the discrete change in position and 
    orientation to the ski for the given timestep duration.
    
    zmeas: actual body z coordinate
    stanceparams: stance controller parameters
    gaitparams: gait controller parameters
    mvref: movement reference
    =#

    Δp = SVector(-mvref.vxyref[1], -mvref.vxyref[2], 1 / stanceparams.ztimeconstant * (mvref.zref - zmeas)) * gaitparams.dt
    ΔR = RotZ(-mvref.wzref * gaitparams.dt)
    return (Δp, ΔR)
end

function stancefootlocations(stancefootlocations::SMatrix{3, 4, Float64}, stanceparams::StanceParams,  gaitparams::GaitParams, mvref::MovementReference)
    zmeas = sum(stancefootlocations[3, :]) / length(stancefootlocations[3, :])
    (Δp, ΔR) = skiincrement(zmeas, stanceparams, mvref, gaitparams)
    incrementedlocations = ΔR * stancefootlocations .+ Δp
    return incrementedlocations
end

function stancefootlocation(stancefootlocation::SVector{3, Float64}, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference)
    #=
    Return the incremented location for a specific foot in stance.
    =#
    zmeas = stancefootlocation[3]
    (Δp, ΔR) = skiincrement(zmeas, stanceparams, mvref, gaitparams)
    incrementedlocation = ΔR * stancefootlocation .+ Δp
    return incrementedlocation
end