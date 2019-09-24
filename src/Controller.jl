using StaticArrays

include("Types.jl")
include("Gait.jl")
include("StanceController.jl")
include("SwingLegController.jl")
include("Kinematics.jl")
include("PupperConfig.jl")

@with_kw mutable struct Controller
    swingparams::SwingParams = SwingParams()
    stanceparams::StanceParams = StanceParams()
    gaitparams::GaitParams = GaitParams()
    mvref::MovementReference = MovementReference(vxyref=SVector{2}(0.0, 0.0), wzref=0.0, zref=-0.18)
    conparams::ControllerParams = ControllerParams()
    robotconfig::PupperConfig = PupperConfig()
    ticks::Int = 0

    footlocations::SMatrix{3, 4, Float64, 12} = stanceparams.defaultstance .+ SVector{3, Float64}(0, 0, mvref.zref)
    jointangles::SMatrix{3, 4, Float64, 12} = fourlegs_inversekinematics(footlocations, robotconfig)
end

function step(ticks::Integer, footlocations::SMatrix{3, 4, Float64}, swingparams::SwingParams, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference, conparams::ControllerParams)
    #=
    Return the foot locations for the next timestep.
    Allocation-free.

    ticks: Time since the system started
    footlocations:: SMatrix
    =#

    contactmodes = contacts(ticks, gaitparams)
    for legindex in 1:4
        contactmode = contactmodes[legindex]
        footloc = SVector{3}(footlocations[:, legindex])
        if contactmode == 1
            newloc = stancefootlocation(footloc, stanceparams, gaitparams, mvref)
        else
            swingprop::Float64 = subphasetime(ticks, gaitparams) / gaitparams.swingticks
            newloc = swingfootlocation(swingprop, footloc, legindex, swingparams, stanceparams, gaitparams, mvref)
        end

        for j in 1:3
            footlocations = setindex(footlocations, newloc[j], LinearIndices(footlocations)[j, legindex])
        end
    end
    return footlocations
end

function stepcontroller!(c::Controller)
    c.footlocations = step(c.ticks, c.footlocations, c.swingparams, c.stanceparams, c.gaitparams, c.mvref, c.conparams)
    c.jointangles = fourlegs_inversekinematics(c.footlocations, c.robotconfig)
    c.ticks += 1
end

function run()
    c = Controller()
    c.mvref = MovementReference(vxyref=SVector{2}(0.2, 0.0), wzref=0.5)

    tf = 6.0
    timesteps = Int(tf / (c.gaitparams.dt))

    footlochistory = zeros(3, 4, timesteps)
    jointanglehistory = zeros(3, 4, timesteps)

    for i in 1:timesteps
        stepcontroller!(c)
        footlochistory[:, :, i] = c.footlocations
        jointanglehistory[:, :, i] = c.jointangles
    end
    return footlochistory, jointanglehistory
end
