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
	filler::Int = 0
end

# Note: Using an MMatrix for the defaultstance or making the struct mutable results in 180k more allocatinons
@with_kw struct StanceParams
    # Time constant in [sec] for the feet in stance to move towards the reference z height
    ztimeconstant::Float64 = 1.0
    Δx::Float64 = 0.1
    Δy::Float64 = 0.06
    defaultstance::SMatrix{3, 4, Float64, 12} = SMatrix{3,4, Float64}(	Δx, -Δy, 0,
                                                                   		Δx,  Δy, 0,
                                                                    	-Δx, -Δy, 0,
                                                                    	-Δx, Δy, 0)
end

@with_kw struct SwingParams
	zclearance::Float64 = 0.03
	Az::SMatrix{5, 5, Float64} = SMatrix{5,5}(	0, 1, 0, 4, 0.5^4,
												0, 1, 0, 3, 0.5^3,
												0, 1, 0, 2, 0.5^2,
												0, 1, 1, 1, 0.5^1,
												1, 1, 0, 0, 0.5^0)
	bz = SVector{5, Float64}(0, 0, 0, 0, zclearance)
	zcoeffs::SVector{5, Float64} = Az \ bz
    alpha::Float64 = 0.5 # Ratio between touchdown distance and total horizontal stance movement
	beta::Float64 = 0.5 # Ratio between touchdown distance and total horizontal stance movement
end

@with_kw struct GaitParams
	# Default values are for a trotting gait
	dt::Float64 = 0.001

	# There are four distinct phases for a trot
	numphases::Int = 4

	# 4 x num_phase matrix of contacts for each gait phase
	contactphases::SMatrix{4, 4, Int, 16} = SMatrix{4, 4, Int, 16}( 1, 1, 1, 1, 
																	1, 0, 0, 1, 
																	1, 1, 1, 1,
																	0, 1, 1, 0)
	overlapticks::Int = Int(0.1 / dt)
	swingticks::Int = Int(0.25 / dt)
	stanceticks::Int = 2 * overlapticks + swingticks
	phasetimes::SVector{4, Int} = SVector(overlapticks, swingticks, overlapticks, swingticks)
	phaselength::Int = 2 * overlapticks + 2 * swingticks
end