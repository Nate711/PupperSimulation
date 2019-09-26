using PiGPIO
using Dates
include("src/Controller.jl")

@with_kw struct PWMParams
    pins::Matrix{Int} = reshape([2,18,2,2,2,2,2,2,2,2,2,2], 3, 4)
    range::Int = 4000
    frequency::Int = 250 # Hz
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
    normalizedangle = (angle - servoparams.neutralangle) / servoparams.anglerange
    pulsewidth_micros = normalizedangle * pwmparams.pulserange + pwmparams.middlepulsewidth
    dutycycle = Int(round(pulsewidth_micros / 1e6 * pwmparams.frequency * pwmparams.range))
    #println(pulsewidth_micros, " ", dutycycle)
    return dutycycle
end

function initializePWM(pi::Pi, pwmparams::PWMParams)
    for legindex in 1:4
        for axis in 1:3
            set_PWM_frequency(pi, pwmparams.pins[axis, legindex], pwmparams.frequency)
            set_PWM_range(pi, pwmparams.pins[axis, legindex], pwmparams.range)
        end
    end
end

function sendservocommands(pi::Pi, pwmparams::PWMParams, servoparams::ServoParams, jointangles::SMatrix{3, 4, Float64, 12})
    for legindex in 1:4
        for axis in 1:3
            if legindex == 1 && axis == 2
                # println(jointangles[axis,legindex])
            end
            dutycycle = angle2dutycycle(jointangles[axis, legindex], pwmparams, servoparams)
            set_PWM_dutycycle(pi, pwmparams.pins[axis, legindex], dutycycle)
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
    controller.mvref = MovementReference(vxyref=SVector(0.2,0.0), zref=-0.15, wzref=0.3)
    controller.swingparams = SwingParams(zclearance=0.02)
    controller.stanceparams = StanceParams(Δx=0.1, Δy=0.09)
    controller.gaitparams = GaitParams(dt=0.01)
    setup(piboard, pwmparams)
    lastloop = Dates.Time(Dates.now())
    for i in 1:1000
        sleep(0.005)
        # TODO: Insert some sort of sleeping or waiting behavior
        loop(piboard, pwmparams, servoparams, controller)
	now = Dates.Time(Dates.now())
	println(Dates.Time(now-lastloop))
        lastloop = now
    end
end
