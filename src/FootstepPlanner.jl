using Parameters

include("Gait.jl")

@with_kw struct FootstepPlannerParams
	# TODO: make sure this next_foot_loc = swing leg next_foot_loc
	next_foot_locs::Vector{Float64} = zeros(12) # TODO: make this a more intelligent initialization for cases where only two feet start active?
end

function nextFootstepLocation!(foot_loc, cur_foot_loc::Vector{T}, v_b::Vector{T}, ω_z::T, gait::GaitParams, next_foot_phase::Int, leg::Int) where {T<:Number}
	# implement body velocity heuristic to get next body relative foot location
  	if leg == 1
		r_i = [WOOFER_CONFIG.LEG_FB, -WOOFER_CONFIG.LEG_LR]
	elseif leg == 2
		r_i = [WOOFER_CONFIG.LEG_FB, WOOFER_CONFIG.LEG_LR]
	elseif leg == 3
		r_i = [-WOOFER_CONFIG.LEG_FB, -WOOFER_CONFIG.LEG_LR]
	else
		r_i = [-WOOFER_CONFIG.LEG_FB, WOOFER_CONFIG.LEG_LR]
	end

	foot_loc[1] = cur_foot_loc[1] + gait.alpha*v_b[1]*gait.phase_times[next_foot_phase] - gait.beta*r_i[2]*ω_z*gait.phase_times[next_foot_phase]
	foot_loc[2] = cur_foot_loc[2] + gait.alpha*v_b[2]*gait.phase_times[next_foot_phase] + gait.beta*r_i[1]*ω_z*gait.phase_times[next_foot_phase]
	foot_loc[3] = cur_foot_loc[3]
end
