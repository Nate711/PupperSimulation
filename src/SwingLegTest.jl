using Parameters
using LinearAlgebra
using SparseArrays
using StaticArrays
using Rotations
using Plots

include("WooferConfig.jl")
include("WooferDynamics.jl")
include("Gait.jl")
include("MPCController.jl")
include("FootstepPlanner.jl")
include("SwingLegController.jl")

trot_gait = GaitParams() # by default initialized to a trot
footstep_config = FootstepPlannerParams()
swing_params = SwingLegParams()

r1 = zeros(3)
r2 = zeros(3)
r3 = zeros(3)
r4 = zeros(3)

joint_pos = zeros(12)
joint_vel = zeros(12)

forwardKinematics!(r1, zeros(3), 1)
forwardKinematics!(r2, zeros(3), 2)
forwardKinematics!(r3, zeros(3), 3)
forwardKinematics!(r4, zeros(3), 4)

t = 0.1

cur_foot_loc = zeros(12)
cur_foot_loc[1:3] = r1
cur_foot_loc[4:6] = r2
cur_foot_loc[7:9] = r3
cur_foot_loc[10:12] = r4

next_foot_loc = zeros(3)
v_b = [0.0, 0.0]

swing_torques = zeros(12)
swing_torque_i = zeros(3)

active_feet_12 = zeros(12)
#
# nextFootstepLocation!(next_foot_loc, r2, v_b, trot_gait, 3)
# swing_params.next_foot_loc[4:6] .= next_foot_loc
#
# t0 = 0.1
# tf = 0.4
# generateFootTrajectory(r2, v_b, t0, tf, 2, swing_params)
#
# p_x = zeros(4)
# p_y = zeros(4)
# p_z = zeros(4)
#
# p_x .= swing_params.foot_trajectories[1:4,2]
# p_y .= swing_params.foot_trajectories[5:8,2]
# p_z .= swing_params.foot_trajectories[9:12,2]
#
# plt = plot3d(1, xlim=(-1, 1), ylim=(-1, 1), zlim=(-1, 1),
# 				title="Relative Foot Trajectory")
#
# for t=t0:0.001:tf
# 	t_vec_xyz = [t^3, t^2, t, 1]
#
# 	x = dot(p_x, t_vec_xyz)
# 	y = dot(p_y, t_vec_xyz)
# 	z = dot(p_z, t_vec_xyz)
#
# 	println(x)
#
# 	push!(plt, x, y, z)
# end
#
# display(plt)
#
# print("x velocity at end of trajectory: ")
# println(dot(p_x, [3*tf^2, 2*tf, 1, 0]))
#
# print("y velocity at end of trajectory: ")
# println(dot(p_y, [3*tf^2, 2*tf, 1, 0]))
#
# print("z velocity at beginning of trajectory: ")
# println(dot(p_z, [3*t0^2, 2*t0, 1, 0]))
#
# print("z velocity at end of trajectory: ")
# println(dot(p_z, [3*tf^2, 2*tf, 1, 0]))

prev_phase = 1
cur_phase = 2
active_feet = trot_gait.contact_phases[:, cur_phase]
coordinateExpander!(active_feet_12, active_feet)

# swing leg
for i in 1:4
	# calculate footstep and generate trajectory (stored in swing params) if needed
	if trot_gait.contact_phases[i, prev_phase] == 1
		if trot_gait.contact_phases[i, cur_phase] == 0
		 nextFootstepLocation!(swing_params.next_foot_loc[(3*(i-1)+1):(3*(i-1)+3)], cur_foot_loc[(3*(i-1)+1):(3*(i-1)+3)], v_b, trot_gait, nextPhase(cur_phase, trot_gait))
		 generateFootTrajectory(cur_foot_loc[(3*(i-1)+1):(3*(i-1)+3)], v_b, t, t+trot_gait.phase_times[cur_phase], i, swing_params)
	  end
	end

	# actually calculate swing torques
	if trot_gait.contact_phases[i, cur_phase] == 0
	  # calculate current foot tip velocity
	  cur_foot_vel_i = legJacobian(joint_pos[(3*(i-1)+1):(3*(i-1)+3)]) * joint_vel[(3*(i-1)+1):(3*(i-1)+3)]

	  calcSwingTorques!(swing_torque_i, cur_foot_loc[(3*(i-1)+1):(3*(i-1)+3)], cur_foot_vel_i, joint_pos[(3*(i-1)+1):(3*(i-1)+3)], t, i, swing_params)
	  swing_torques[(3*(i-1)+1):(3*(i-1)+3)] .= swing_torque_i
	end
end

swing_torques
