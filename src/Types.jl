using Parameters
using LinearAlgebra

@with_kw mutable struct MovementCommand
    vxydes::MVector{2, Float64} = MVector(0, 0)
    wzdes::Float64 = 0
    zdes::Float64 = -0.18
end

# # Note: Mutable structs are allocated on the heap
# @with_kw mutable struct MovementReference
#     vxyref::MVector{2, Float64} = MVector(0, 0)
#     wzref::Float64 = 0
#     zref::Float64 = -0.18
# end

# Note: Mutable structs are allocated on the heap
@with_kw struct MovementReference
    vxyref::SVector{2, Float64} = SVector(0, 0)
    wzref::Float64 = 0
    zref::Float64 = -0.18
end

@with_kw struct ControllerParams
    dt::Float64 = 0.01
end

# Note: Using an MMatrix for the defaultstance or making the struct mutable results in 180k more allocatinons
@with_kw struct StanceParams
    # Time constant in [sec] for the feet in stance to move towards the reference z height
    ztimeconstant::Float64 = 1.0
    Δx::Float64 = 0.1
    Δy::Float64 = 0.06
    defaultstance::SMatrix{3, 4, Float64} = SMatrix{3,4, Float64}(  Δx, -Δy, 0,
                                                                    Δx,  Δy, 0,
                                                                    -Δx, -Δy, 0,
                                                                    -Δx, -Δy, 0)
end

@with_kw struct SwingParams
	z_clearance::Float64 = 0.02
	Az::SMatrix{5, 5, Float64} = SMatrix{5,5}(	0, 1, 0, 4, 0.5^4,
												0, 1, 0, 3, 0.5^3,
												0, 1, 0, 2, 0.5^2,
												0, 1, 1, 1, 0.5^1,
												1, 1, 0, 0, 0.5^0)
	bz = SVector{5, Float64}(0, 0, 0, 0, z_clearance)
	zcoeffs::SVector{5, Float64} = Az \ bz
    alpha::Float64 = 0.5 # Ratio between touchdown distance and total horizontal stance movement
	beta::Float64 = 0.5 # Ratio between touchdown distance and total horizontal stance movement
end

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
end