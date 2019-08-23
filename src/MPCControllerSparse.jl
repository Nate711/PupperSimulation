using OSQP
using ControlSystems
using SparseArrays
using LinearAlgebra

include("WooferConfig.jl")

# using Parametron # TODO: use this

@with_kw struct MPCControllerParams
	# gravity
	g::Float64 = 9.81

	min_vert_force::Float64 = 1.0
	max_vert_force::Float64 = 133.0

	# inverse of inertia tensor in body frame
	J_inv::Diagonal{Float64}

	# discretization length
	dt::Float64

	# planning horizon length
	N::Int64

	A_c::SparseMatrixCSC{Float64, Int64}

	cont_sys::Matrix{Float64} = zeros(22, 22)
	disc_sys::Matrix{Float64} = zeros(22, 22)
	B_c::Matrix{Float64} = zeros(10, 12)
	r_hat::Matrix{Float64} = zeros(3,3)

	A_d::Matrix{Float64} = zeros(10, 10)
	B_d::Array{Float64,2} = zeros(10,12)

	C_dense::Matrix{Float64}
	C_i::Matrix{Float64}
	# C_sparse::SparseMatrixCSC{Float64, Int64} # TODO: Add this
	lb::Vector{Float64}
	ub::Vector{Float64}

	k::Vector{Float64}
	r::Vector{Float64}
	H::Array{Float64, 2}
	V::Array{Float64, 2} = zeros(9,9)

	u_ref::Vector{Float64}
	z_ref::Vector{Float64} = zeros(22*N)
	z_result::Vector{Float64} = zeros(22*N)

	# OSQP model
	prob::OSQP.Model
	first_run::Vector{Bool} = [true]
	P::SparseMatrixCSC{Float64, Int64} = zeros(22*N, 22*N)
	q::Vector{Float64} = zeros(22*N)
end

function initMPCControllerConfig(dt::Number, N::Integer)
	J_inv = inv(WOOFER_CONFIG.INERTIA)

	A_c = zeros(10,10)
	A_c[1:3, 6:8] = Matrix{Float64}(I, 3, 3)
	A_c[6, 10] = -1
	A_c = sparse(A_c)

	prob = OSQP.Model()

	# TODO: construct constant constraint matrices here
	C_i = zeros(20, 12)
	mu = 1.5
	min_vert_force = 1
	max_vert_force = 133

	for i in 1:4
		# fz >= 0
		C_i[(i-1)*5+1, (i-1)*3+3] = 1
		# ufz+fx >= 0
		C_i[(i-1)*5+2, (i-1)*3+1] = 1
		C_i[(i-1)*5+2, (i-1)*3+3] = mu
		# fx-ufz <= 0
		C_i[(i-1)*5+3, (i-1)*3+1] = 1
		C_i[(i-1)*5+3, (i-1)*3+3] = -mu
		# ufz+fy >= 0
		C_i[(i-1)*5+4, (i-1)*3+2] = 1
		C_i[(i-1)*5+4, (i-1)*3+3] = mu
		# fy-ufz <= 0
		C_i[(i-1)*5+5, (i-1)*3+2] = 1
		C_i[(i-1)*5+5, (i-1)*3+3] = -mu
	end

	lb_i = zeros(30)
	ub_i = zeros(30)

	for i in 1:4
		# fz >= 0
		lb_i[(i-1)*5+11] = min_vert_force
		ub_i[(i-1)*5+11] = max_vert_force
		# ufz+fx >= 0
		lb_i[(i-1)*5+12] = 0
		ub_i[(i-1)*5+12] = Inf
		# fx-ufz <= 0
		lb_i[(i-1)*5+13] = -Inf
		ub_i[(i-1)*5+13] = 0
		# ufz+fy >= 0
		lb_i[(i-1)*5+14] = 0
		ub_i[(i-1)*5+14] = Inf
		# fy-ufz >= 0
		lb_i[(i-1)*5+15] = -Inf
		ub_i[(i-1)*5+15] = 0
	end

	C_dense = zeros((20 + 10)*N, 22*N)

	lb = repeat(lb_i, N)
	ub = repeat(ub_i, N)

	k = [50, 1, 1, 1, 1, 1, 1, 1, 1, 0]
	r = 1e-10*ones(12)

	H = Diagonal(repeat([r..., k...], N))

	u_ref = [0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0]*WOOFER_CONFIG.MASS*9.81/4

	# mpcConfig = MPCControllerParams(prob=prob, J_inv=J_inv, dt=dt, N=N, Q=Q, R=R, A_c=A_c, C_dense=C_dense, C_i=C_i, k, lb=lb, ub=ub, min_vert_force=min_vert_force, max_vert_force=max_vert_force)
	mpcConfig = MPCControllerParams(J_inv=J_inv, dt=dt, N=N, H=H, A_c=A_c, C_dense=C_dense, C_i=C_i, lb=lb, ub=ub, k=k[1:9], r=r, prob=prob, u_ref=u_ref)
	return mpcConfig
end

function generateReferenceTrajectory!(x_ref::Array{T, 2}, x_curr::Vector{T}, x_des::Vector{T}, mpc_config::MPCControllerParams) where {T<:Number}
	# how agressively the MPC will attempt to get to desired state
	stiffness = 1.0

	N_ = floor(stiffness*mpc_config.N)

	x_diff = 1/N_ * (x_des - x_curr)
	for i in 1:mpc_config.N
		if i<N_
			x_ref[:, i] .= x_curr + i*x_diff
		else
			x_ref[:, i] .= x_des
		end
	end
end

function skewSymmetricMatrix!(A::Matrix, a::Vector)
	A[1,2] = -a[3]
	A[1,3] = a[2]
	A[2,1] = a[3]
	A[2,3] = -a[1]
	A[3,1] = -a[2]
	A[3,2] = a[1]
end

function solveFootForces!(forces::Vector{T}, x0::Vector{T}, x_ref::Array{T, 2}, contacts::Array{Int,2}, foot_locs::Array{T,2}, mpc_config::MPCControllerParams, woofer_config::WooferConfig) where {T<:Number}
	# x_ref: 10xN matrix of state reference trajectory
	# contacts: 4xN matrix of foot contacts over the planning horizon
	# foot_locs: 12xN matrix of foot location in body frame over planning horizon

	# print(mpc_config.N)

	for i in 1:mpc_config.N
		## construct continuous time B matrix
		for j in 1:4
			if contacts[j,i] == 1
				mpc_config.B_c[4:6, (3*(j-1)+1):(3*(j-1)+3)] .= 1/woofer_config.MASS*Matrix{Float64}(I, 3, 3)

				skewSymmetricMatrix!(mpc_config.r_hat, foot_locs[(3*(j-1)+1):(3*(j-1)+3),i])
				mpc_config.B_c[7:9, (3*(j-1)+1):(3*(j-1)+3)] .= mpc_config.J_inv*mpc_config.r_hat

				mpc_config.lb[(30*(i-1)+(j-1)*5+11)] = mpc_config.min_vert_force
				mpc_config.ub[(30*(i-1)+(j-1)*5+11)] = mpc_config.max_vert_force
			else
				mpc_config.B_c[4:9, (3*(j-1)+1):(3*(j-1)+3)] .= zeros(6, 3)

				mpc_config.lb[(30*(i-1)+(j-1)*5+11)] = 0
				mpc_config.ub[(30*(i-1)+(j-1)*5+11)] = 0
			end
		end

		# get the discretized system matrices via ZOH approximation:
		mpc_config.cont_sys[1:10, 1:10] .= mpc_config.A_c
		mpc_config.cont_sys[1:10, 11:22] .= mpc_config.B_c

		mpc_config.disc_sys .= exp(mpc_config.cont_sys*mpc_config.dt)
		if i == 1
			mpc_config.A_d .= mpc_config.disc_sys[1:10, 1:10]
			mpc_config.B_d .= mpc_config.disc_sys[1:10, 11:22]
			# mpc_config.A_d = (Matrix{Float64}(I, 10, 10) + mpc_config.A_c*mpc_config.dt + (mpc_config.A_c*mpc_config.dt)^2/2 + (mpc_config.A_c*mpc_config.dt)^3/6 + (mpc_config.A_c*mpc_config.dt)^4/24 + (mpc_config.A_c*mpc_config.dt)^5/120 + (mpc_config.A_c*mpc_config.dt)^6/720)
		end
		mpc_config.B_d .= mpc_config.disc_sys[1:10, 11:22]
		# mpc_config.B_d .= mpc_config.dt*mpc_config.A_d*mpc_config.B_c

		## add row of C
		mpc_config.C_dense[(30*(i-1)+1):(30*(i-1)+10), (22*(i-1)+1):(22*(i-1)+12)] .= mpc_config.B_d
		mpc_config.C_dense[(30*(i-1)+1):(30*(i-1)+10), (22*(i-1)+13):(22*(i-1)+22)] .= -Matrix{Float64}(I, 10, 10)
		if i != 1
			mpc_config.C_dense[(30*(i-1)+1):(30*(i-1)+10), (22*(i-2)+13):(22*(i-2)+22)] .= mpc_config.A_d
			mpc_config.lb[(30*(i-1)+1):(30*(i-1)+10)] .= zeros(10) # possibly redundant?
			mpc_config.ub[(30*(i-1)+1):(30*(i-1)+10)] .= zeros(10) # possibly redundant?
		else
			mpc_config.lb[(30*(i-1)+1):(30*(i-1)+10)] .= -mpc_config.A_d*x0
			mpc_config.ub[(30*(i-1)+1):(30*(i-1)+10)] .= -mpc_config.A_d*x0
		end
		# friction constraints
		mpc_config.C_dense[(30*(i-1)+11):(30*(i-1)+30), (22*(i-1)+1):(22*(i-1)+12)] .= mpc_config.C_i

		# control reference
		mpc_config.z_ref[22*(i-1)+1:22*(i-1)+12] .= mpc_config.u_ref
		# state reference
		mpc_config.z_ref[22*(i-1)+13:22*(i-1)+22] .= x_ref[10*(i-1)+1:10*(i-1)+10]
	end

	# set the terminal constraint to be LQR cost to go
	mpc_config.V .= dare(	mpc_config.A_d[1:9, 1:9], mpc_config.B_d[1:9,:],
							Diagonal(mpc_config.k), Diagonal(mpc_config.r))
	mpc_config.H[22*(mpc_config.N - 1)+13:22*(mpc_config.N - 1)+21, 22*(mpc_config.N - 1)+13:22*(mpc_config.N - 1)+21] = mpc_config.V
	# mpc_config.H[22*(mpc_config.N - 1)+13:22*(mpc_config.N - 1)+21, 22*(mpc_config.N - 1)+13:22*(mpc_config.N - 1)+21] .= Array(Diagonal(mpc_config.k))

	# FIXME: allocation here
	C_sparse = sparse(mpc_config.C_dense)

	mpc_config.P .= sparse(2*mpc_config.H)
	mpc_config.q .= sparse(-2*mpc_config.H*mpc_config.z_ref)

	# warm starting the solver!
	if mpc_config.first_run[1]
		OSQP.setup!(mpc_config.prob; P=mpc_config.P, q=mpc_config.q, A=C_sparse, l=mpc_config.lb, u=mpc_config.ub, verbose=false, warm_start=true)
		mpc_config.first_run[1] = false
	else
		OSQP.update!(mpc_config.prob; Px=triu(mpc_config.P).nzval, Ax=C_sparse.nzval, q=mpc_config.q)
	end

	results = OSQP.solve!(mpc_config.prob)

	mpc_config.z_result .= results.x

	forces .= (results.x)[1:12]

	# print((results.x)[22*(mpc_config.N-1)+13:22*(mpc_config.N-1)+22])
	# cost_total = (mpc_config.z_result - mpc_config.z_ref)' * mpc_config.H * (mpc_config.z_result - mpc_config.z_ref)
	# println(cost_total)
	#
	# cost_stage = ((mpc_config.z_result - mpc_config.z_ref)[1:12])' * mpc_config.H[1:12, 1:12] * ((mpc_config.z_result - mpc_config.z_ref)[1:12])
	# println(cost_stage)
	#
	# cost_togo = ((mpc_config.z_result - mpc_config.z_ref)[13:22])' * mpc_config.H[13:22, 13:22] * ((mpc_config.z_result - mpc_config.z_ref)[13:22])
	# println(cost_togo)
end
