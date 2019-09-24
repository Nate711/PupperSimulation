using PiGPIO
include("src/Controller.jl")

@with_kw struct PWMParams
    pins::Matrix{Int} = reshape([2,18,2,2,2,2,2,2,2,2,2,2], 4, 3)
    range::Int = 10000
    frequency::Int = 320 # Hz
    min::Int = 1000 # minimum servo pulse width [us]
    max::Int = 2000 # maximum servo pulse width [us]
    middlepulsewidth::Int = (min + max) / 2
    pulserange::Int = max - min
end

@with_kw struct ServoParams
    anglerange::Float64 = 160.0 / 180.0 * pi
    neutralangle::Float64 = 0.0
end

function angle2dutycycle(angle::Float64, pwmparams::PWMParams, servoparams::ServoParams)
    pulsewidth_micros = (angle - servoparams.neutralangle) / servoparams.anglerange * pwmparams.pulserange + pwmparams.middlepulsewidth
    # println(pulsewidth_micros)
    return Int(round(pulsewidth_micros / 1e6 * pwmparams.frequency * pwmparams.range))
end

function initializePWM(pi::Pi, pwmparams::PWMParams)
    for legindex in 1:4
        for axis in 1:3
            set_PWM_frequency(pi, pwmparams.pins[legindex, axis], pwmparams.frequency)
            set_PWM_range(pi, pwmparams.pins[legindex, axis], pwmparams.range)
        end
    end
end

function sendservocommands(pi::Pi, pwmparams::PWMParams, servoparams::ServoParams, jointangles::SMatrix{3, 4, Float64, 12})
    for legindex in 1:4
        for axis in 1:3
            if legindex == 1 && axis == 3
                println(jointangles[axis,legindex])
            end
            dutycycle = angle2dutycycle(jointangles[axis, legindex], pwmparams, servoparams)
            set_PWM_dutycycle(pi, pwmparams.pins[legindex, axis], dutycycle)
        end
    end
end

function setup(pi::Pi, pwmparams::PWMParams)
    initializePWM(pi, pwmparams::PWMParams)
end

function loop(pi::Pi, pwmparams::PWMParams, servoparams, controller::Controller)
    stepcontroller!(controller)
    sendservocommands(pi, pwmparams, servoparams, controller.jointangles)
end

function main()
    piboard = Pi()
    pwmparams = PWMParams()
    servoparams = ServoParams()

    controller = Controller()
    controller.mvref = MovementReference(vxyref=SVector(0.2,0.0))
    setup(piboard, pwmparams)

    for i in 1:2000
        sleep(0.01)
        # TODO: Insert some sort of sleeping or waiting behavior
        loop(piboard, pwmparams, servoparams, controller)
        println()
    end
end
