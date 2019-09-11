using ForwardDiff
using Parameters
using Rotations
include("WooferConfig.jl")

function legSide(i::Integer)
	# Returns 1.0 if the leg is on the right and -1.0 if the leg is on the left
	@assert i >= 1 && i <= 4
	if i == 2 || i == 4
		return 1.0
	else
		return -1.0
	end
end

function legFrontBack(i::Integer)
	@assert i>=1 && i <= 4
	if i == 1 || i == 2
		return 1.0
	else
		return -1.0
	end
end

function assertValidLeg(i::Integer)
	@assert i >= 1 && i <= 4
	return nothing
end

function legForwardKinematics!(r_body::Vector{Float64}, α::Vector{Float64}, i::Integer, config::WooferConfig)
	assertValidLeg(i)

	beta = α[1]
	theta = α[2]
	r = α[3]
	if i == 2 || i == 4
		unrotated = [0, config.ABDUCTION_OFFSET::Float64, -config.LEG_L::Float64 + r]
	else
		unrotated = [0, -config.ABDUCTION_OFFSET::Float64, -config.LEG_L::Float64 + r]
	end

	# RotXY is equiv to an intrinsic rotation first around the x axis, and then the new y axis
	r_body .= RotXY(beta, theta) * unrotated
end

function legForwardKinematics(α::SVector{3, Float64}, i::Integer, config::WooferConfig)
	#=
	Given the joint angles, return the vector from the hip to the foot in the body frame.

	α: Joint angles. 
	i: Leg number. 1 = front right, 2 = front left, 3 = back right, 4 = back left.

	return: Vector from hip to foot in body coordinates.
	=#
	assertValidLeg(i)

	beta = α[1]
	theta = α[2]
	r = α[3]
	unrotated_leg = SVector(0.0, legSide(i) * config.ABDUCTION_OFFSET, -config.LEG_L + r)

	# RotXY is equiv to an intrinsic rotation first around the x axis, and then the new y axis
	return RotXY(beta, theta) * unrotated_leg
end

# function forwardKinematicsAll!(r_body::Vector{Float64}, α::Vector{Float64}, abduction_offset=0)
# 	for i in 1:4
# 		# leg pointing straight down
# 		unrotated = [0, abduction_offset, -WOOFER_CONFIG.LEG_L + α[3*(i-1)+3]]

# 		# vector from leg hub to foot in body coordinates
# 		p_i = RotXY( α[3*(i-1)+1], α[3*(i-1)+2]) * unrotated
# 		if i==1
# 			r_body[1:3] .= p_i + [WOOFER_CONFIG.LEG_FB, -WOOFER_CONFIG.LEG_LR, 0]
# 		elseif i==2
# 			r_body[4:6] .= p_i + [WOOFER_CONFIG.LEG_FB, WOOFER_CONFIG.LEG_LR, 0]
# 		elseif i==3
# 			r_body[7:9] .= p_i + [-WOOFER_CONFIG.LEG_FB, -WOOFER_CONFIG.LEG_LR, 0]
# 		else
# 			r_body[10:12] .= p_i + [-WOOFER_CONFIG.LEG_FB, WOOFER_CONFIG.LEG_LR, 0]
# 		end
# 	end
# end

function explicitLegInverseKinematics(r_body_foot::SVector{3, Float64}, i::Integer, config::WooferConfig)
	assertValidLeg(i)

	# Unpack vector from body to foot
	x, y, z = r_body_foot

	# Distance from the hip to the foot, projected into the y-z plane
	R_body_foot_yz = (y ^ 2 + z ^ 2) ^ 0.5

	# Distance from the leg's forward/back point of rotation to the foot
	R_hip_foot_yz = (R_body_foot_yz ^ 2 - config.ABDUCTION_OFFSET ^ 2) ^ 0.5

	# Ensure that the target point is reachable
	@assert abs(config.ABDUCTION_OFFSET) <= R_body_foot_yz

	# Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
	# For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
	ϕ = acos(legSide(i) * config.ABDUCTION_OFFSET / R_body_foot_yz)

	# Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis 
	θ_ = atan(z, y)

	# Ab/adduction angle, relative to the positive y-axis
	α1 = ϕ + θ_

	# θ: Angle between the tilted negative z-axis and the hip-to-foot vector
	θ = atan(-x, R_hip_foot_yz)

	# Forward/back angle: Angle between the tilted negative z-axis and the hip-to-foot vector
	α2 = θ

	# Distance between the hip and foot
	R_hip_foot = (R_hip_foot_yz ^ 2 + x ^ 2) ^ 0.5

	# Leg extension
	α3 = -R_hip_foot + config.LEG_L

	# Code if using the full five-bar linkage model
	# # Angle between the leg links L1 and L2
	# β = acos((config.LEG_L1 ^ 2 + config.LEG_L2 ^ 2 - R_hip_foot ^ 2) / (2 * config.LEG_L1 * config.LEG_L2))

	# # Angle between the leg center of action and L1
	# ψ = acos((config.LEG_L1 ^ 2 + R_hip_foot ^ 2 - config.LEG_L2) / (2 * config.LEG_L1 * R_hip_foot))

	# # Angle of the first link relative to the tilted negative z axis
	# α2 .= θ - ψ

	# # Angle of the second link relative to the tilted negative z axis
	# α3 .= α2 + (π - β)
	return SVector(α1, α2, α3)
end

function allLegsInverseKinematics(r_body_foot::SMatrix{3, 4, Float64}, config::WooferConfig)
	α = zeros(MMatrix{3, 4, Float64})
	for i in 1:4
		body_offset = SVector{3,Float64}(legFrontBack(i) * config.LEG_FB, legSide(i) * config.LEG_LR, 0.0)
		α[:, i] = explicitLegInverseKinematics(r_body_foot[:, i] - body_offset, i, config)
	end
	s = SMatrix(α)
	return s
end