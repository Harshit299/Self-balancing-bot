# # PID gains
# Kp_angle = 39.4  # Proportional gain for the angle
# Ki_angle = 920.7  # Integral gain for the angle
# Kd_angle = 0.035  # Derivative gain for the angle

# # Initialize error accumulations
# integral_angle = 0
# integral_vel = 0
# previous_error_angle = 0
# previous_error_vel = 0

# def sysCall_init():
#     global left_joint, right_joint, bot_body

#     sim = require('sim')

#     # Get handles for the joints and the robot body
#     left_joint = sim.getObject('/left_joint')
#     right_joint = sim.getObject('/right_joint')
#     bot_body = sim.getObject('/body')

#     # Initialize joints to zero velocity
#     sim.setJointTargetVelocity(left_joint, 0)
#     sim.setJointTargetVelocity(right_joint, 0)

# def sysCall_sensing():
#     global bot_body

#     # Get the current orientation of the robot body
#     # Assuming bot_body gives access to orientation angles
#     orientation = sim.getObjectOrientation(bot_body, -1)  # Get orientation relative to the world
#     pos = sim.getObjectPosition(bot_body, -1)

#     # Orientation[0] gives tilt along the x-axis (forward/backward tilt)
#     return orientation[0], pos  # Return tilt angles (x, y, z)

# def sysCall_actuation():
#     global integral_angle, integral_vel, previous_error_angle, previous_error_vel

#     # Get the current tilt angle and angular velocity of the robot
#     angle, _ = sysCall_sensing()
#     print("Tilt angle (Z-axis):", angle)
    
#     # Compute PID control for the angle (balance control)
#     dt = sim.getSimulationTimeStep()  # Get the simulation time step

#     # Calculate error between the desired upright position and the current angle
#     error_angle = 0 - angle  # We want the tilt angle to be 0 (upright position)

#     total_error = error_angle
    
#     # Proportional term
#     P_angle = Kp_angle * total_error

#     # Integral term
#     integral_angle += total_error * dt
#     I_angle = Ki_angle * integral_angle

#     # Derivative term
#     derivative_angle = (total_error - previous_error_angle) / dt
#     D_angle = Kd_angle * derivative_angle

#     # Total control output for angle
#     control = P_angle + I_angle + D_angle

#     # Store current error for the next derivative calculation
#     previous_error_angle = total_error


#     # Apply control signal to both motors to balance the robot
#     # If the robot leans forward, drive motors backward, and vice versa
    
#     if total_error > 0.085 or total_error < -0.085:
#         sim.setJointTargetVelocity(left_joint, control)
#         sim.setJointTargetVelocity(right_joint, control)


# def sysCall_cleanup():
#     global left_joint, right_joint

#     # Reset motor velocities
#     sim.setJointTargetVelocity(left_joint, 0)
#     sim.setJointTargetVelocity(right_joint, 0)


























# PID gains
Kp_angle = 65.6  # Proportional gain for the angle
Ki_angle = 949.4  # Integral gain for the angle
Kd_angle = 0.035  # Derivative gain for the angle

# Initialize error accumulations
integral_angle = 0
previous_error_angle = 0

def sysCall_init():
    global left_joint, right_joint, bot_body

    sim = require('sim')

    # Get handles for the joints and the robot body
    left_joint = sim.getObject('/left_joint')
    right_joint = sim.getObject('/right_joint')
    bot_body = sim.getObject('/body')

    # Initialize joints to zero velocity
    sim.setJointTargetVelocity(left_joint, 0)
    sim.setJointTargetVelocity(right_joint, 0)

def sysCall_sensing():
    global bot_body

    # Get the current orientation of the robot body
    orientation = sim.getObjectOrientation(bot_body, -1)  # Get orientation relative to the world
    pos = sim.getObjectPosition(bot_body, -1)

    # Orientation[0] gives tilt along the x-axis (forward/backward tilt)
    return orientation[0], pos  # Return tilt angles (x, y, z)

def sysCall_actuation():
    global integral_angle, previous_error_angle

    # Get the current tilt angle and angular velocity of the robot
    angle, _ = sysCall_sensing()
    print("Tilt angle (Z-axis):", angle)
    
    # Compute PID control for the angle (balance control)
    dt = sim.getSimulationTimeStep()  # Get the simulation time step

    # Calculate error between the desired upright position and the current angle
    error_angle = 0 - angle  # We want the tilt angle to be 0 (upright position)

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

    ############### Keyboard Input ##############
    message, data, data2 = sim.getSimulatorMessage()

    if (message == sim.message_keypress):
        if (data[0] == 2007):  # forward (up arrow)
            sim.setJointTargetVelocity(left_joint, control + 0.05)
            sim.setJointTargetVelocity(right_joint, control + 0.05)

        elif (data[0] == 2008):  # backward (down arrow)
            sim.setJointTargetVelocity(left_joint, control - 0.5)
            sim.setJointTargetVelocity(right_joint, control - 0.5)

        elif (data[0] == 2009):  # left (left arrow)
            # To rotate left, the left wheel should move backward, and the right wheel forward
            sim.setJointTargetVelocity(left_joint, control + 2.3)
            sim.setJointTargetVelocity(right_joint, control - 2.3)

        elif (data[0] == 2010):  # right (right arrow)
            # To rotate right, the left wheel should move forward, and the right wheel backward
            sim.setJointTargetVelocity(left_joint, control - 2.3)
            sim.setJointTargetVelocity(right_joint, control + 2.3)

    else:
        if total_error > 0.085 or total_error < -0.085:
            sim.setJointTargetVelocity(left_joint, control)
            sim.setJointTargetVelocity(right_joint, control)


def sysCall_cleanup():
    global left_joint, right_joint

    # Reset motor velocities
    sim.setJointTargetVelocity(left_joint, 0)
    sim.setJointTargetVelocity(right_joint, 0)