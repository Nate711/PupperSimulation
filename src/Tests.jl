using LinearAlgebra

include("WooferDynamics.jl")
include("WooferConfig.jl")

function round_(a, dec)
    return map(x -> round(x, digits=dec), a)
end

function testInverseKinematicsExplicit!()
    println("\nTesting Inverse Kinematics")
    function testHelper(r, alpha_true, i; do_assert=true)
        eps = 1e-6
        α = zeros(3)
        inverseKinematicsExplicit!(α, r, i)
        println("Leg ", i, ": r: ", r, " -> α: ", α)
        if do_assert
            @assert norm(α - alpha_true) < eps
        end
    end
    
    c = WOOFER_CONFIG.LEG_L/sqrt(2)
    offset = WOOFER_CONFIG.ABDUCTION_OFFSET
    testHelper([0, offset, -0.125], [0, 0, 0], 2)
    testHelper([c, offset, -c], [0, -pi/4, 0], 2)
    testHelper([-c, offset, -c], [0, pi/4, 0], 2)
    testHelper([0, c, -c], missing, 2, do_assert=false)

    testHelper([-c, -offset, -c], [0, pi/4, 0], 1)
    testHelper([WOOFER_CONFIG.LEG_L * sqrt(3)/2, offset, -WOOFER_CONFIG.LEG_L / 2], [0, -pi/3, 0], 2)
end

function testForwardKinematics!()
    println("\nTesting Forward Kinematics")
    function testHelper(alpha, r_true, i; do_assert=true)
        eps = 1e-6
        r = zeros(3)
        legForwardKinematics!(r, alpha, i)
        println("Leg ", i, ": α: ", alpha, " -> r: ", r)
        if do_assert
            @assert norm(r_true - r) < eps
        end
    end

    l = WOOFER_CONFIG.LEG_L
    offset = WOOFER_CONFIG.ABDUCTION_OFFSET
    testHelper([0.0, 0.0, 0.0], [0, offset, -l], 2)
    testHelper([0.0, pi/4, 0.0], missing, 2, do_assert=false)
end

function testForwardInverseAgreeance()
    println("\nTest forward/inverse consistency")
    eps = 1e-6
    for i in 1:10
        alpha = [rand()-0.5, rand()-0.5, (rand()-0.5)*0.05]
        leg = rand(1:4)
        r = zeros(3)
        legForwardKinematics!(r, alpha, leg)
        alpha_prime = zeros(3)
        inverseKinematicsExplicit!(alpha_prime, r, leg)
        println("Leg ", leg, ": α: ", round_(alpha, 3), " -> r_body_foot: ", round_(r, 3), " -> α': ", round_(alpha_prime, 3))
        @assert norm(alpha_prime - alpha) < eps
    end
end

testInverseKinematicsExplicit!()
testForwardKinematics!()
testForwardInverseAgreeance()
@time testInverseKinematicsExplicit!()
@time testForwardKinematics!()
@time testForwardInverseAgreeance()