@with_kw struct WooferConfig
	MASS::Float64 = 1.00

	# Robot joint limits
	MAX_JOINT_TORQUE::Float64 = 1.0
	MAX_LEG_FORCE::Float64 = 15.0
	REVOLUTE_RANGE::Float64 = 1.57
	PRISMATIC_RANGE::Float64 = 0.125

	# Robot geometry
	LEG_FB::Float64 = 0.10  # front-back distance from center line to leg axis
	LEG_LR::Float64 = 0.0569  # left-right distance from center line to leg plane
	LEG_L::Float64  = 0.125

	# Used only for the 4-bar linkage model of the leg
	LEG_L1::Float64 = 0.1235
	LEG_L2::Float64 = 0.125
	
	ABDUCTION_OFFSET::Float64 = 0.027  # distance from abduction axis to leg
	FOOT_RADIUS::Float64 = 0.01

	L::Float64 = 0.276
	W::Float64 = 0.100
	T::Float64 = 0.050
	Ix::Float64 = MASS/12 * (W^2 + T^2)
	Iy::Float64 = MASS/12 * (L^2 + T^2)
	Iz::Float64 = MASS/12 * (L^2 + W^2)
	INERTIA::Diagonal{Float64} = Diagonal([Ix, Iy, Iz])
end

WOOFER_CONFIG = WooferConfig()
