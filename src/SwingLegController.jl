include("StanceController.jl")
include("Gait.jl")
include("Types.jl")

# TODO: This function still allocates??
function raibert_tdlocations(swingparams::SwingParams, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference)
	#=
	Use the raibert heuristic to find the touchdown locations for all legs as if they were all in swing
	=#

	# Contruct the 'skis' defined by p and R
	p2 = swingparams.alpha * gaitparams.stancetime * mvref.vxyref
	p = SVector(p2[1], p2[2], 0.0)
	Θ = swingparams.beta * gaitparams.stancetime * mvref.wzref
	R = RotZ(Θ)

	# Apply the ski transformation to the default foot locations
	tdlocations = R * stanceparams.defaultstance .+ p
	return tdlocations
end

function swingheight(swingt::Number, swingparams::SwingParams, gaitparams::GaitParams)
	swingphase = swingt / gaitparams.swingtime
	timevec = SVector{5}(swingphase^4, swingphase^3, swingphase^2, swingphase, 1)
	sheight = timevec' * swingparams.zcoeffs # vertical offset relative to the stance default
	return sheight
end

function swingfootlocations(swingt::Number, footlocations::SMatrix{3, 4, Float64}, swingparams::SwingParams, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference, p::ControllerParams)
	#=
	Return the incremental position update for the legs as if they were all in swing
	
	swingt: Time since start in swing phase in seconds
	footlocations: Current foot locations
	=#

	@assert swingt >= 0 && swingt <= gaitparams.swingtime

	# That sheight is the height where the stance feet should be at the *given time*, 
	# while tdlocations is the location that the feet should be at at the *end* of the swing period.
	sheight = swingheight(swingt, swingparams, gaitparams)
	tdlocations = raibert_tdlocations(swingparams, stanceparams, gaitparams, mvref)
	timeleft = gaitparams.swingtime - swingt
	v = (tdlocations - footlocations) / timeleft .* SVector{3}(1, 0, 0)
	Δfootlocations = v * p.dt
	sheight_matrix = SMatrix{3, 4, Float64}(0, 0, sheight + mvref.zref,
											0, 0, sheight + mvref.zref,
											0, 0, sheight + mvref.zref,
											0, 0, sheight + mvref.zref)
	incrementedlocations = sheight_matrix + Δfootlocations
	return incrementedlocations
end
