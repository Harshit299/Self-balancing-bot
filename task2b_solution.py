import numpy as np

# PID gains
Kp_angle = 109.6  # Proportional gain for the angle
Ki_angle = 1069.7  # Integral gain for the angle
Kd_angle = 0.03105  # Derivative gain for the angle

# Initialize error accumulations
integral_angle = 0
previous_error_angle = 0

def sysCall_init():
    global left_joint, right_joint, bot_body, prismatic_joint, arm

    sim = require('sim')

    # Get handles for the joints and the robot body
    left_joint = sim.getObject('/left_joint')
    right_joint = sim.getObject('/right_joint')
    bot_body = sim.getObject('/body')
    prismatic_joint = sim.getObject('/Prismatic_joint')
    arm = sim.getObject('/arm_joint')

    # Initialize joints to zero velocity
    sim.setJointTargetVelocity(left_joint, 0)
    sim.setJointTargetVelocity(right_joint, 0)
    

def sysCall_sensing():
    global bot_body

    # Get the current orientation of the robot body
    orientation = sim.getObjectOrientation(bot_body, -1)  
 
    return orientation[0], orientation[1], orientation[2]  

def calc_error(eq_angle, dt):
    global previous_error_angle, integral_angle

    angle_x, angle_y, angle_z = sysCall_sensing()

    error_angle = eq_angle - (np.cos(angle_z) * angle_x + np.sin(angle_z) * angle_y)

    total_error = error_angle
    
    # Proportional term
    P_angle = Kp_angle * total_error

    # Integral term
    integral_angle += total_error * dt
    I_angle = Ki_angle * integral_angle

    # Derivative term
    derivative_angle = (total_error - previous_error_angle) / dt
    D_angle = Kd_angle * derivative_angle

    # Total control output for angle
    control = P_angle + I_angle + D_angle

    # Store current error for the next derivative calculation
    previous_error_angle = total_error

    return control, total_error

def sysCall_actuation():
    global integral_angle, previous_error_angle, eq_angle

    # Compute PID control for the angle (balance control)
    dt = sim.getSimulationTimeStep() 

    ############### Keyboard Input ##############
    message, data, _ = sim.getSimulatorMessage()

    if (message == sim.message_keypress):

        #*************** Forward *******************************8
        if (data[0] == 2008):  # forward (up arrow)
            
            control, _ = calc_error(0.05, dt)

            sim.setJointTargetVelocity(left_joint, control + 0.3)
            sim.setJointTargetVelocity(right_joint, control + 0.3)

        #***************** Backward *********************************
        elif (data[0] == 2007):  # backward (down arrow)
            
            control, _ = calc_error(-0.05, dt)

            sim.setJointTargetVelocity(left_joint, control - 0.3)
            sim.setJointTargetVelocity(right_joint, control - 0.3)

        #****************** Left **************************************
        elif (data[0] == 2009):  # left (left arrow)
            
            control, _ = calc_error(0, dt)

            sim.setJointTargetVelocity(left_joint, control + 3)
            sim.setJointTargetVelocity(right_joint, control - 3)

        #*************** Right *********************************
        elif (data[0] == 2010):  # right (right arrow)            
            
            control, _ = calc_error(0, dt)

            sim.setJointTargetVelocity(left_joint, control - 3)
            sim.setJointTargetVelocity(right_joint, control + 3)
            
            
        #**************** Q key ********************************
        elif data[0] == 113:  # "q" key - close gripper
            sim.setJointTargetVelocity(prismatic_joint, 0.09)
            
            control,total_error = calc_error(0, dt)

            if total_error > 0.035 or total_error < -0.035:
                sim.setJointTargetVelocity(left_joint, control)
                sim.setJointTargetVelocity(right_joint, control)

        #**************** E key ********************************
        elif data[0] == 101:  # "e" key - open gripper
            sim.setJointTargetVelocity(prismatic_joint, -0.09)
            
            control, total_error = calc_error(0, dt)

            if total_error > 0.035 or total_error < -0.035:
                sim.setJointTargetVelocity(left_joint, control)
                sim.setJointTargetVelocity(right_joint, control)
            
        #**************** R key ********************************
        elif data[0] == 114:  # "r" key - down gripper
            sim.setJointTargetVelocity(arm, 2.3)
            
            control, total_error = calc_error(0, dt)

            if total_error > 0.035 or total_error < -0.035:
                sim.setJointTargetVelocity(left_joint, control)
                sim.setJointTargetVelocity(right_joint, control)
            
        #**************** F key ******************************** 
        elif data[0] == 102:  # "f" key - up gripper
            sim.setJointTargetVelocity(arm, -2.3)
            
            control, total_error = calc_error(0, dt)

            if total_error > 0.035 or total_error < -0.035:
                sim.setJointTargetVelocity(left_joint, control)
                sim.setJointTargetVelocity(right_joint, control)

    #*******************************************************************************
    else:
        control, total_error = calc_error(0, dt)

        if total_error > 0.035 or total_error < -0.035:
            sim.setJointTargetVelocity(left_joint, control)
            sim.setJointTargetVelocity(right_joint, control)


        sim.setJointTargetVelocity(prismatic_joint, 0)
        sim.setJointTargetVelocity(arm, 0)


def sysCall_cleanup():
    global left_joint, right_joint

    # Reset motor velocities
    sim.setJointTargetVelocity(left_joint, 0)
    sim.setJointTargetVelocity(right_joint, 0)