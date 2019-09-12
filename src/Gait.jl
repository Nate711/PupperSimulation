using Parameters
using StaticArrays

include("Types.jl")

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
