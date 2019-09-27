using PiGPIO
using Dates
include("src/Controller.jl")
include("src/HardwareInterface.jl")

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
    
    initializePWM(piboard, pwmparams)
    
    lastloop = Dates.Time(Dates.now())
    now = lastloop
    for i in 1:1000
        stepcontroller!(controller)
        sendservocommands(piboard, pwmparams, servoparams, controller.jointangles)

        while now - lastloop < controller.gaitparams.dt
            now = Dates.Time(Dates.now())
        end
	    println("Time since last loop: ", now - lastloop)
        lastloop = now
    end
end
