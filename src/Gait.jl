using Parameters

@with_kw struct GaitParams
	# Default == Trot

	# add in a cyclic array for the phases?
	num_phases::Int64 = 4

	# 4xnum_phase array of contacts for each gait phase
	contact_phases::Array{Int64} = [1 1 1 0; 1 0 1 1; 1 0 1 1; 1 1 1 0]

	phase_times::Vector{Float64} = [0.1, 0.25, 0.1, 0.25]

	phase_length::Float64 = sum(phase_times)
	alpha::Float64 = 0.5
	beta::Float64 = 0
end

function getPhase(t::AbstractFloat, gait_params::GaitParams)
	phase_time = t % gait_params.phase_length

	for i in 1:gait_params.num_phases
		if phase_time < sum(gait_params.phase_times[1:i])
			return i
		end
	end
end

function nextPhase(phase::Integer, gait_params::GaitParams)
	if (phase == gait_params.num_phases)
		return 1
	else
		return phase+1
	end
end

function coordinateExpander!(expanded::Vector, compact::Vector)
	expanded[1:3] .= compact[1]
	expanded[4:6] .= compact[2]
	expanded[7:9] .= compact[3]
	expanded[10:12] .= compact[4]
end
