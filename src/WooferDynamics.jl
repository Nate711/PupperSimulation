using ForwardDiff
using Parameters
using Rotations
include("WooferConfig.jl")

function legJacobian(αᵢ::Vector{Float64}, abduction_offset::Float64 = 0)
	# ported from WooferDynamics.py
	i = [1, 0, 0]
	j = [0, 1, 0]
	k = [0, 0, 1]

	beta = αᵢ[1]
	theta = αᵢ[2]
	r = αᵢ[3]

	# leg pointing straight down
	unrotated = [0, abduction_offset, -WOOFER_CONFIG.LEG_L + r]
	# vector from leg hub to foot in body coordinates
	p = RotYX(theta, beta)*unrotated

	# directions of positive joint movement
	theta_axis = RotX(beta)*j
	beta_axis = i
	radial_axis = k

	J = zeros(3,3)

	# dpdbeta
	J[:,1] .= cross(beta_axis, p)
	# dpdtheta
	J[:,2] .= cross(theta_axis, p)
	# dpdr
	J[:,3] .= RotYX(theta, beta)*radial_axis

	return J

end

function forwardKinematics2(α::Vector, i::Integer)
	if i==1
		r = [WOOFER_CONFIG.LEG_FB, 	-WOOFER_CONFIG.LEG_LR, 0]
	elseif i==2
		r = [WOOFER_CONFIG.LEG_FB, 	 WOOFER_CONFIG.LEG_LR, 0]
	elseif i==3
		r = [-WOOFER_CONFIG.LEG_FB, 	-WOOFER_CONFIG.LEG_LR, 0]
	else
		r = [-WOOFER_CONFIG.LEG_FB, 	 WOOFER_CONFIG.LEG_LR, 0]
	end

	r[1] += -(WOOFER_CONFIG.LEG_L + WOOFER_CONFIG.FOOT_RADIUS + α[3])*sin(α[2])
	r[2] += (WOOFER_CONFIG.LEG_L + WOOFER_CONFIG.FOOT_RADIUS + α[3])*cos(α[2])*sin(α[1])
	r[3] += -(WOOFER_CONFIG.LEG_L + WOOFER_CONFIG.FOOT_RADIUS + α[3])*cos(α[2])*cos(α[1])

	return r
end

function legJacobian2(α::Vector{Float64})
	J = zeros(3,3)
	J[1,1] = 0
	J[1,2] = -(WOOFER_CONFIG.LEG_L + WOOFER_CONFIG.FOOT_RADIUS + α[3])*cos(α[2])
	J[1,3] = -sin(α[2])
	J[2,1] = (WOOFER_CONFIG.LEG_L + WOOFER_CONFIG.FOOT_RADIUS + α[3])*cos(α[2])*cos(α[1])
	J[2,2] = -(WOOFER_CONFIG.LEG_L + WOOFER_CONFIG.FOOT_RADIUS + α[3])*sin(α[2])*sin(α[1])
	J[2,3] = cos(α[2])*sin(α[1])
	J[3,1] = (WOOFER_CONFIG.LEG_L + WOOFER_CONFIG.FOOT_RADIUS + α[3])*cos(α[2])*sin(α[1])
	J[3,2] = (WOOFER_CONFIG.LEG_L + WOOFER_CONFIG.FOOT_RADIUS + α[3])*sin(α[2])*cos(α[1])
	J[3,3] = -cos(α[2])*cos(α[1])

	return J
end

function force2Torque!(τ::Vector{Float64}, f::Vector{Float64}, α::Vector{Float64})
	τ[1:3] .= transpose(legJacobian(α[1:3]))*f[1:3]
	τ[4:6] .= transpose(legJacobian(α[4:6]))*f[4:6]
	τ[7:9] .= transpose(legJacobian(α[7:9]))*f[7:9]
	τ[10:12] .= transpose(legJacobian(α[10:12]))*f[10:12]
end

function legForwardKinematics!(r_body::Vector{T}, α::Vector{T}, i::Int) where {T<:Number}
	#=
	Given the joint angles, return the vector from the hip to the foot in the body frame.

	r_body: Vector from hip to foot in body coordinates. Output parameter.
	α: Joint angles. 
	i: Leg number. 1 = front right, 2 = front left, 3 = back right, 4 = back left
	=#
	@assert i in [1, 2, 3, 4]

	beta = α[1]
	theta = α[2]
	r = α[3]
	if i == 2 || i == 4
		unrotated = [0, WOOFER_CONFIG.ABDUCTION_OFFSET, -WOOFER_CONFIG.LEG_L + r]
	else
		unrotated = [0, -WOOFER_CONFIG.ABDUCTION_OFFSET, -WOOFER_CONFIG.LEG_L + r]
	end

	# RotXY is equiv to an intrinsic rotation first around the x axis, and then the new y axis
	r_body .= RotXY(beta, theta) * unrotated
end

function forwardKinematics!(r_body::Vector{Float64}, α::Vector{Float64}, i::Int, abduction_offset=0)
	p = zeros(3)
	legForwardKinematics!(p, α, i)

	if i == 1
		r_body .= p + [WOOFER_CONFIG.LEG_FB, -WOOFER_CONFIG.LEG_LR, 0]
	elseif i == 2
		r_body .= p + [WOOFER_CONFIG.LEG_FB, WOOFER_CONFIG.LEG_LR, 0]
	elseif i == 3
		r_body .= p + [-WOOFER_CONFIG.LEG_FB, -WOOFER_CONFIG.LEG_LR, 0]
	else
		r_body .= p + [-WOOFER_CONFIG.LEG_FB, WOOFER_CONFIG.LEG_LR, 0]
	end
end

function forwardKinematicsAll!(r_body::Vector{Float64}, α::Vector{Float64}, abduction_offset=0)
	for i in 1:4
		# leg pointing straight down
		unrotated = [0, abduction_offset, -WOOFER_CONFIG.LEG_L + α[3*(i-1)+3]]

		# vector from leg hub to foot in body coordinates
		p_i = RotXY( α[3*(i-1)+1], α[3*(i-1)+2]) * unrotated
		if i==1
			r_body[1:3] .= p_i + [WOOFER_CONFIG.LEG_FB, -WOOFER_CONFIG.LEG_LR, 0]
		elseif i==2
			r_body[4:6] .= p_i + [WOOFER_CONFIG.LEG_FB, WOOFER_CONFIG.LEG_LR, 0]
		elseif i==3
			r_body[7:9] .= p_i + [-WOOFER_CONFIG.LEG_FB, -WOOFER_CONFIG.LEG_LR, 0]
		else
			r_body[10:12] .= p_i + [-WOOFER_CONFIG.LEG_FB, WOOFER_CONFIG.LEG_LR, 0]
		end
	end
end

function inverseKinematicsExplicit!(α::Vector{T}, r_body_foot::Vector{T}, i::Int) where {T<:Number}
	@assert i in [1, 2, 3, 4]
	
	# Unpack vector from body to foot
	x, y, z = r_body_foot

	# Distance from the hip to the foot, projected into the y-z plane
	R_body_foot_yz = (y ^ 2 + z ^ 2) ^ 0.5

	# Distance from the leg's forward/back point of rotation to the foot
	R_hip_foot_yz = (R_body_foot_yz ^ 2 - WOOFER_CONFIG.ABDUCTION_OFFSET ^ 2) ^ 0.5

	# ϕ: The interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
	# Ensure that the target point is reachable
	@assert abs(WOOFER_CONFIG.ABDUCTION_OFFSET) <= R_body_foot_yz

	# For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
	if i == 2 || i == 4
		ϕ = acos(WOOFER_CONFIG.ABDUCTION_OFFSET / R_body_foot_yz)
	else
		ϕ = acos(-WOOFER_CONFIG.ABDUCTION_OFFSET / R_body_foot_yz)
	end

	# θ_: The angle of the y-z projection of the hip-to-foot vector relative to the positive y-axis 
	θ_ = atan(z, y)

	# Ab/adduction angle, relative to the positive y-axis
	α[1] = ϕ + θ_

	# θ: Angle between the tilted negative z-axis and the hip-to-foot vector
	θ = atan(-x, R_hip_foot_yz)

	# Forward/back angle: Angle between the tilted negative z-axis and the hip-to-foot vector
	α[2] = θ

	# Distance between the hip and foot
	R_hip_foot = (R_hip_foot_yz ^ 2 + x ^ 2) ^ 0.5

	# Leg extension
	α[3] = -R_hip_foot + WOOFER_CONFIG.LEG_L

	# Code if using the full five-bar linkage model
	# # Angle between the leg links L1 and L2
	# β = acos((WOOFER_CONFIG.LEG_L1 ^ 2 + WOOFER_CONFIG.LEG_L2 ^ 2 - R_hip_foot ^ 2) / (2 * WOOFER_CONFIG.LEG_L1 * WOOFER_CONFIG.LEG_L2))

	# # Angle between the leg center of action and L1
	# ψ = acos((WOOFER_CONFIG.LEG_L1 ^ 2 + R_hip_foot ^ 2 - WOOFER_CONFIG.LEG_L2) / (2 * WOOFER_CONFIG.LEG_L1 * R_hip_foot))

	# # Angle of the first link relative to the tilted negative z axis
	# α[2] .= θ - ψ

	# # Angle of the second link relative to the tilted negative z axis
	# α[3] .= α[2] + (π - β)
end

	
function inverseKinematics!(α::Vector{T}, r_body::Vector{T}, i::Int, n::Int = 5, abduction_offset::Number=0) where {T<:Number}
	#=
	calculate joint angles via Newton's method
	α: initial guess (that will be updated in place)
	n: max number of iterations

	# TODO:
	=#

	r_guess = zeros(3)
	eps = 1e-2

	for k=1:n
		forwardKinematics!(r_guess, zeros(3), i)
		if norm(r_body - r_guess) < eps
			break
		end
		α .= pinv(legJacobian(α))* (r_body - r_guess) + α
	end

	print(norm(r_body - r_guess))

end
