using LinearAlgebra
using Profile
using StaticArrays

include("Kinematics.jl")
include("PupperConfig.jl")
include("Gait.jl")
include("StanceController.jl")
include("SwingLegController.jl")
include("Types.jl")

function round_(a, dec)
    return map(x -> round(x, digits=dec), a)
end

function testInverseKinematicsExplicit!()
    println("\n-------------- Testing Inverse Kinematics -----------")
    config = PupperConfig()
    println("\nTesting Inverse Kinematics")
    function testHelper(r, alpha_true, i; do_assert=true)
        eps = 1e-6
        @time α = explicitLegInverseKinematics(r, i, config)
        println("Leg ", i, ": r: ", r, " -> α: ", α)
        if do_assert
            @assert norm(α - alpha_true) < eps
        end
    end
    
    c = config.LEG_L/sqrt(2)
    offset = config.ABDUCTION_OFFSET
    testHelper(SVector(0, offset, -0.125), SVector(0, 0, 0), 2)
    testHelper(SVector(c, offset, -c), SVector(0, -pi/4, 0), 2)
    testHelper(SVector(-c, offset, -c), SVector(0, pi/4, 0), 2)
    testHelper(SVector(0, c, -c), missing, 2, do_assert=false)

    testHelper(SVector(-c, -offset, -c), [0, pi/4, 0], 1)
    testHelper(SVector(config.LEG_L * sqrt(3)/2, offset, -config.LEG_L / 2), SVector(0, -pi/3, 0), 2)
end

function testForwardKinematics!()
    println("\n-------------- Testing Forward Kinematics -----------")
    config = PupperConfig()
    println("\nTesting Forward Kinematics")
    function testHelper(alpha, r_true, i; do_assert=true)
        eps = 1e-6
        r = zeros(3)
        println("Vectors")
        a = [alpha.data...]
        @time legForwardKinematics!(r, a, i, config)
        println("SVectors")
        @time r = legForwardKinematics(alpha, i, config)
        println("Leg ", i, ": α: ", alpha, " -> r: ", r)
        if do_assert
            @assert norm(r_true - r) < eps
        end
    end

    l = config.LEG_L
    offset = config.ABDUCTION_OFFSET
    testHelper(SVector{3}([0.0, 0.0, 0.0]), SVector{3}([0, offset, -l]), 2)
    testHelper(SVector{3}([0.0, pi/4, 0.0]), missing, 2, do_assert=false)
    # testHelper([0.0, 0.0, 0.0], [0, offset, -l], 2)
    # testHelper([0.0, pi/4, 0.0], missing, 2, do_assert=false)
end

function testForwardInverseAgreeance()
    println("\n-------------- Testing Forward/Inverse Consistency -----------")
    config = PupperConfig()
    println("\nTest forward/inverse consistency")
    eps = 1e-6
    for i in 1:10
        alpha = SVector(rand()-0.5, rand()-0.5, (rand()-0.5)*0.05)
        leg = rand(1:4)
        @time r = legForwardKinematics(alpha, leg, config)
        # @code_warntype legForwardKinematics!(r, alpha, leg, config)
        @time alpha_prime = explicitLegInverseKinematics(r, leg, config)
        # @code_warntype inverseKinematicsExplicit!(alpha_prime, r, leg, config)
        println("Leg ", leg, ": α: ", round_(alpha, 3), " -> r_body_foot: ", round_(r, 3), " -> α': ", round_(alpha_prime, 3))
        @assert norm(alpha_prime - alpha) < eps
    end
end

function testStaticArrays()
    println("\n-------------- Comparing Arrays -----------")
    function helper(a::MVector{3, Float64})
        a[1] = 0.5
    end
    function helper2(a::SVector{3, Float64})
        b = SVector(0.5, a[2], a[3])
        return b
    end
    function helper3(a::Vector{Float64})
        a[1] = 0.5
        return nothing
    end
    function helper4(a::Vector{Float64})
        b = [0.5, a[2], a[3]]
        return b
    end
    a0 = MVector(1.0,2,3)
    println("MVector")
    @time helper(a0)
    println("Return SVector")
    @time a = helper2(SVector(1.0,2,3))
    a1 = [1.0,2,3]
    println("Modify vector")
    @time helper3(a1)
    println("Return new vector")
    @time a = helper4(a1)
    return nothing
end

function testAllInverseKinematics()
    println("\n-------------- Testing Four Leg Inverse Kinematics -----------")
    function helper(r_body, alpha_true; do_assert=true)        
        println("Timing for allLegsInverseKinematics")
        config = PupperConfig()
        @time alpha = allLegsInverseKinematics(SMatrix(r_body), config)
        println("r: ", r_body, " -> α: ", alpha)
        
        if do_assert
            @assert norm(alpha - alpha_true) < 1e-10
        end
    end
    config = PupperConfig()
    f = config.LEG_FB
    l = config.LEG_LR
    s = -0.125
    o = config.ABDUCTION_OFFSET
    r_body = MMatrix{3,4}(zeros(3,4))
    r_body[:,1] = [f, -l-o, s]
    r_body[:,2] = [f, l+o, s]
    r_body[:,3] = [-f, -l-o, s]
    r_body[:,4] = [-f, l+o, s]

    helper(r_body, zeros(3,4))
    helper(SMatrix{3,4}(zeros(3,4)), missing, do_assert=false)
end

function testKinematics()
    testInverseKinematicsExplicit!()
    testForwardKinematics!()
    testForwardInverseAgreeance()
    testAllInverseKinematics()
end

function testGait()
    println("\n-------------- Testing Gait -----------")
    p = GaitParams()
    # println("Gait params=",p)
    t = 1.2
    println("Timing for getPhase")
    @time ph = getPhase(t, p)
    # @code_warntype getPhase(t, p)
    println("t=",t," phase=",ph)
    @assert ph == 4
    @assert getPhase(0.0, p) == 1
    
    println("Timing for getContacts")
    @time c = getContacts(t, p)
    # @code_warntype getContacts(t, p)
    @assert typeof(c) == SArray{Tuple{4},Int64,1,4}
    println("t=", t, " contacts=", c)
end

function TestStanceController()
    println("\n-------------- Testing Stance Controller -----------")
    c = StanceParams()
    cparams = ControllerParams()
    @time dp, dR = skiincrement(SVector(0.0, 0.0, 0.0), 0.0, -0.2, -0.1, c, cparams)
    @assert norm(dR - I(3)) < 1e-10
    @assert norm(dp - [0, 0, -1e-3]) < 1e-10
    @time dp, dR = skiincrement(SVector(0.0, 1.0, 0.0), -3.0, -0.2, -0.1, c, cparams)
end

function TestSwingLegController()
    println("\n-------------- Testing Swing Leg Controller -----------")
    swp = SwingParams()
    stp = StanceParams()
    gp = GaitParams()
    p = ControllerParams()
    println("Timing for swingheight:")
    @time z = swingheight(0.125, swp, gp)
    println("z clearance at t=1/2swingtime =>",z)
    @assert abs(z - swp.z_clearance) < 1e-10

    println("Timing for swingheight:")
    @time z = swingheight(0, swp, gp)
    println("Z clearance at t=0 =>",z)
    @assert abs(z) < 1e-10

    mvref = MovementReference(SVector(1.0, 0.0), 0, -0.18)
    println("Timing for raibert tdlocations:")
    @time l = raibert_tdlocations(swp, stp, gp, mvref)
    target = stp.defaultstance .+ [gp.stancetime*0.5*1, 0, 0]
    println("Touchdown locations =>", l, " <?=> ", target)
    @assert norm(l - target) <= 1e-10
    
    fcurrent = SMatrix{3, 4, Float64}(stp.defaultstance)
    mvref = MovementReference()
    tswing = 0.125
    println("Timing for swingfootlocation increment")
    @time l = swingfootlocations(tswing, fcurrent, swp, stp, gp, mvref, p)
    println(l)
end

testGait()
testKinematics()
TestStanceController()
testStaticArrays()
TestSwingLegController()