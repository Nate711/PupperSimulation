include("Types.jl")
using PiGPIO

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