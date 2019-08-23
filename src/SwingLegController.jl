@with_kw struct SwingLegParams
	foot_trajectories::Array{Float64, 2} = zeros(12, 4)
	next_foot_loc::Vector{Float64} = zeros(12)

	z_rel_des::Float64 = -0.2

	wn_cart::Float64 = 20
	zeta_cart::Float64 = 0.8
	kp_cart::Float64 = wn_cart^2
	kd_cart::Float64 = 2*wn_cart*zeta_cart
end

function generateFootTrajectory(foot_loc_cur::Vector{T}, v_b::Vector{T}, t0::T, tf::T, i::Int, swing_params::SwingLegParams) where {T<:Number}
	#=
	Generate a body relative trajectory for the ith foot via spline interpolation
	This function is called when the phase is switched to a no contact phase

	Updated foot location must be put into SwingLegParams instantiation
	=#

	eps = 0.05

	# generate cubic spline in x,y to get body relative foot trajectory
	A = [t0^3 	t0^2 	t0 	1;
		 tf^3 	tf^2 	tf 	1;
		 3*t0^2 2*t0^2 	1 	0;
		 3*tf^2 2*tf^2 	1 	0]

	# TODO: add in omega cross term here? probably doesn't matter...
	b_x = [foot_loc_cur[1], swing_params.next_foot_loc[3*(i-1)+1], -v_b[1], -v_b[1]]
	b_y = [foot_loc_cur[2], swing_params.next_foot_loc[3*(i-1)+2], -v_b[2], -v_b[2]]

	# generate cubic spline in z to enforce height constraint and terminal velocity constraint
	A_z =  [t0^3				t0^2				t0				1;
			tf^3				tf^2				tf				1;
			(0.5*(tf+t0))^3		(0.5*(tf+t0))^2		(0.5*(tf+t0))	1;
			3*tf^2 				2*tf^2 				1 				0]

	b_z = [foot_loc_cur[3], swing_params.next_foot_loc[3*(i-1)+3], swing_params.z_rel_des, -eps]

	swing_params.foot_trajectories[1:4, i] 	.= A\b_x
	swing_params.foot_trajectories[5:8, i] 	.= A\b_y
	swing_params.foot_trajectories[9:12, i] .= A_z\b_z

end

function calcSwingTorques!(swing_torques::Vector{T}, cur_pos::Vector{T}, cur_vel::Vector{T}, α::Vector{T}, t::T, i::Integer, swing_params::SwingLegParams) where {T<:Number}
	#=
	PD cartesian controller around swing leg trajectory
	=#

	t_p = [t^3, t^2, t, 1]
	t_v = [3*t^2, 2*t^2, 1, 0]

	r_des = [dot(swing_params.foot_trajectories[1:4,i], t_p),
			 dot(swing_params.foot_trajectories[5:8,i], t_p),
			 dot(swing_params.foot_trajectories[9:12,i], t_p)]

	v_des = [dot(swing_params.foot_trajectories[1:4,i], t_v),
			 dot(swing_params.foot_trajectories[5:8,i], t_v),
			 dot(swing_params.foot_trajectories[9:12,i], t_v)]

	swing_torques .= legJacobian(α)' * (swing_params.kp_cart*(r_des - cur_pos) + swing_params.kd_cart*(v_des - cur_vel))
end
