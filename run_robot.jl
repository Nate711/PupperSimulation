using PiGPIO
include("src/Controller.jl")

@with_kw struct PWMParams
    pins::Vector{Int} = reshape(1:12, 3, 4)
    range::Int = 10000
    frequency::Int = 320
end

function initializePWM(pi::Pi, pwmparams::PWMParams)
    for legindex in 1:4
        for axis in 1:3
            set_PWM_frequency(pi, pwmparams.pins[legindex, axis], pwmparams.frequency)
            set_PWM_range(pi, pwmparams.pins[legindex, axis], pwmparams.range)
        end
    end
end

function sendservocommands(pi::Pi, pwmparams::PWMParams, jointangles::SMatrix{3, 4, Float64, 12})
    for legindex in 1:4
        for axis in 1:3
            dutycycle = jointangles[axis, legindex] * pwmparams.range
            set_PWM_dutycycle(pi, pwmparams.pins[legindex, axis], dutycycle)
        end
    end
end

function setup(pi::Pi, pwmparams::PWMParams)
    initializePWM(pi, pwmparams::PWMParams)
end

function loop(pi::Pi, pwmparams::PWMParams, controller::Controller)
    stepcontroller!(controller)
    sendservocommands(pi, pwmparams, controller.jointangles)

end

function main()
    pi = Pi()
    pwmparams = PWMParams()

    controller = Controller()
    setup(pi, pwmparams)

    while true
        # TODO: Insert some sort of sleeping or waiting behavior
        loop(pi, pwmparams, controller)
    end
end

main()