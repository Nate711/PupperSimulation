using Parameters
using StaticArrays

@with_kw struct GaitParams
	# Default values are for a trotting gait
	# There are four distinct phases for a trot
	num_phases::Int64 = 4

	# 4 x num_phase matrix of contacts for each gait phase
	contact_phases::SMatrix{4, 4, Int64, 16} = SMatrix{4, 4, Int64, 16}(1, 1, 1, 1, 
																		1, 0, 0, 1, 
																		1, 1, 1, 1,
																		0, 1, 1, 0)
	overlaptime::Float64 = 0.1
	swingtime::Float64 = 0.25
	stancetime::Float64 = 2 * overlaptime + swingtime
	phase_times::SVector{4, Float64} = SVector(overlaptime, swingtime, overlaptime, swingtime)
	phase_length::Float64 = 2 * overlaptime + 2 * swingtime
	alpha::Float64 = 0.5
	beta::Float64 = 0
end

function getPhase(t::Float64, gait_params::GaitParams)
	#=
	Given the time, return the phase number.
	t: Time
	gait_params: Gait parameters specifying values including the phase times
	=#

	phase_time = t % gait_params.phase_length
	phase_sum = 0
	for i in 1:gait_params.num_phases
		phase_sum = phase_sum + gait_params.phase_times[i]
		if phase_time < phase_sum
			return i
		end
	end
	@assert false
	return 0
end

function getContacts(t::Real, gait_params::GaitParams)
	#=
	Given the time, return an array specifying the feet in contact. 
	t: Time
	gait_params: Gait parameters specifying values including phase times and contacts
	=#

	return gait_params.contact_phases[:, getPhase(t, gait_params)]
end
