using Parameters
using StaticArrays

include("Types.jl")

function phaseindex(ticks::Integer, gaitparams::GaitParams)
	#=
	Given the time, return the phase number.
	ticks: Ticks since program start where the time elapsed is ticks * dt
	gait_params: Gait parameters specifying values including the phase times
	=#

	phasetime = ticks % gaitparams.phaselength
	phasesum = 0
	for i in 1:gaitparams.numphases
		phasesum = phasesum + gaitparams.phasetimes[i]
		if phasetime < phasesum
			return i::Int
		end
	end
	@assert false
	return 0::Int
end

function subphasetime(ticks::Integer, gaitparams::GaitParams)
	phasetime = ticks % gaitparams.phaselength
	phasesum::Int = 0
	subphaset::Int = 0
	for i in 1:gaitparams.numphases
		phasesum = phasesum + gaitparams.phasetimes[i]
		if phasetime < phasesum
			subphaset = phasetime - phasesum + gaitparams.phasetimes[i]
			break
		end
	end
	return subphaset
end

function contacts(ticks::Integer, gait_params::GaitParams)
	#=
	Given the time, return an array specifying the feet in contact. 
	t: Time
	gait_params: Gait parameters specifying values including phase times and contacts
	=#

	return gait_params.contactphases[:, phaseindex(ticks, gait_params)]
end
