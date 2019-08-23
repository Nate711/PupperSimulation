import os
import shutil
from os.path import expanduser
from PupperConfig import PUPPER_CONFIG, ENVIRONMENT_CONFIG, SOLVER_CONFIG


def Parse():
    ###### ROBOT PARAMETERS #####

    ## Solver params ##
    pupper_timestep = ENVIRONMENT_CONFIG.DT  # timestep
    pupper_joint_solref = "0.001 1"  # time constant and damping ratio for joints
    pupper_joint_solimp = "0.9 0.95 0.001"  # joint constraint parameters

    pupper_geom_solref = "0.005 2"  # time constant and damping ratio for geom contacts
    pupper_geom_solimp = "0.9 0.95 0.001"  # geometry contact parameters

    pupper_armature = PUPPER_CONFIG.ARMATURE  # armature for joints [kgm2]

    ## Geometry params ##
    pupper_leg_radius = PUPPER_CONFIG.FOOT_RADIUS  # radius of leg capsule
    pupper_friction = ENVIRONMENT_CONFIG.MU  # friction between legs and ground
    pupper_half_size = "%s %s %s" % (
        PUPPER_CONFIG.L / 2,
        PUPPER_CONFIG.W / 2,
        PUPPER_CONFIG.T / 2,
    )  # half-size of body box
    pupper_leg_geom = "0 0 0 0 0 %s" % (-PUPPER_CONFIG.LEG_L)  # to-from leg geometry
    pupper_start_position = "0 0 %s" % (
        PUPPER_CONFIG.LEG_L + pupper_leg_radius
    )  # Initial position of the robot torso
    pupper_hip_box = "%s %s %s" % (
        PUPPER_CONFIG.HIP_L,
        PUPPER_CONFIG.HIP_W,
        PUPPER_CONFIG.HIP_T,
    )  # Size of the box representing the hip

    pupper_force_geom = "0 0 -0.34"

    ## Mass/Inertia Params ##
    pupper_frame_mass = PUPPER_CONFIG.FRAME_MASS

    pupper_frame_inertia = "0.0065733 0.074011 0.077763"
    pupper_module_inertia = "0.002449 0.005043 0.006616 -0.001784 -.00002 -0.000007"
    pupper_leg_inertia = "0.003575 0.006356 0.002973 -0.0001326 -0.0001079 -0.0002538"

    ## Joint params ##
    pupper_joint_range = "%s %s" % (
        -PUPPER_CONFIG.REVOLUTE_RANGE,
        PUPPER_CONFIG.REVOLUTE_RANGE,
    )  # joint range in rads for angular joints
    pupper_joint_force_range = "%s %s" % (
        -PUPPER_CONFIG.MAX_JOINT_TORQUE,
        PUPPER_CONFIG.MAX_JOINT_TORQUE,
    )  # force range for ab/ad and forward/back angular joints
    pupper_ext_force_range = "%s %s" % (
        -PUPPER_CONFIG.MAX_LEG_FORCE,
        PUPPER_CONFIG.MAX_LEG_FORCE,
    )  # force range for radial/extension joint
    pupper_ext_range = "%s %s" % (
        -PUPPER_CONFIG.PRISMATIC_RANGE,
        PUPPER_CONFIG.PRISMATIC_RANGE,
    )  # joint range for radial/extension joint
    pupper_rad_damping = 15  # damping on radial/extension joint [N/m/s]
    pupper_joint_damping = 0.2  # damping on ab/ad and f/b angular joints [Nm/rad/s]

    ## Sensor Noise Parameters ##
    pupper_accel_noise = 0.01
    pupper_encoder_noise = 0.001
    pupper_gyro_noise = 0.02
    pupper_encoder_vel_noise = 0.01
    pupper_force_noise = 0

    ###### FILE PATHS  #####

    dir_path = os.path.dirname(os.path.realpath(__file__))
    in_file = dir_path + "/pupper.xml"
    out_file = dir_path + "/pupper_out.xml"

    ### Parse the xml ###
    print("Parsing MuJoCo XML file:")
    print("Input xml: %s" % in_file)
    print("Output xml: %s" % out_file)

    with open(in_file, "r") as file:
        filedata = file.read()

    #### Replace variable names with values ####

    # Solver specs
    filedata = filedata.replace("pupper_timestep", str(pupper_timestep))
    filedata = filedata.replace("pupper_joint_solref", str(pupper_joint_solref))
    filedata = filedata.replace("pupper_geom_solref", str(pupper_geom_solref))
    filedata = filedata.replace("pupper_friction", str(pupper_friction))
    filedata = filedata.replace("pupper_armature", str(pupper_armature))
    filedata = filedata.replace("pupper_joint_solimp", str(pupper_joint_solimp))
    filedata = filedata.replace("pupper_geom_solimp", str(pupper_geom_solimp))

    # Joint specs
    filedata = filedata.replace("pupper_ext_force_range", str(pupper_ext_force_range))
    filedata = filedata.replace("pupper_ext_range", str(pupper_ext_range))
    filedata = filedata.replace("pupper_joint_range", str(pupper_joint_range))
    filedata = filedata.replace(
        "pupper_joint_force_range", str(pupper_joint_force_range)
    )
    filedata = filedata.replace("pupper_rad_damping", str(pupper_rad_damping))
    filedata = filedata.replace("pupper_joint_damping", str(pupper_joint_damping))

    # Geometry specs
    filedata = filedata.replace("pupper_hip_bo", str(pupper_hip_bo))
    filedata = filedata.replace("pupper_frame_mass", str(PUPPER_CONFIG.FRAME_MASS))
    filedata = filedata.replace("pupper_module_mass", str(PUPPER_CONFIG.MODULE_MASS))
    filedata = filedata.replace("pupper_leg_mass", str(PUPPER_CONFIG.LEG_MASS))
    filedata = filedata.replace("pupper_frame_inertia", str(pupper_frame_inertia))
    filedata = filedata.replace("pupper_module_inertia", str(pupper_module_inertia))
    filedata = filedata.replace("pupper_leg_inertia", str(pupper_leg_inertia))
    filedata = filedata.replace("pupper_leg_radius", str(pupper_leg_radius))
    filedata = filedata.replace("pupper_half_size", str(pupper_half_size))
    filedata = filedata.replace("pupper_leg_fb", str(PUPPER_CONFIG.LEG_FB))
    filedata = filedata.replace("pupper_leg_lr", str(PUPPER_CONFIG.LEG_LR))
    filedata = filedata.replace("pupper_leg_geom", str(pupper_leg_geom))
    filedata = filedata.replace("pupper_start_position", str(pupper_start_position))
    filedata = filedata.replace("pupper_force_geom", str(pupper_force_geom))
    filedata = filedata.replace("pupper_hip_box", str(pupper_hip_box))
    filedata = filedata.replace("pupper_hip_offset", str(PUPPER_CONFIG.HIP_OFFSET))
    filedata = filedata.replace(
        "pupper_abduction_offset", str(PUPPER_CONFIG.ABDUCTION_OFFSET)
    )

    # Sensor noise
    filedata = filedata.replace("pupper_accel_noise", str(pupper_accel_noise))
    filedata = filedata.replace("pupper_gyro_noise", str(pupper_gyro_noise))
    filedata = filedata.replace("pupper_encoder_noise", str(pupper_encoder_noise))
    filedata = filedata.replace(
        "pupper_encoder_vel_noise", str(pupper_encoder_vel_noise)
    )
    filedata = filedata.replace("pupper_force_noise", str(pupper_force_noise))

    # Write the xml file
    with open(out_file, "w") as file:
        file.write(filedata)


Parse()
print("DONE")
