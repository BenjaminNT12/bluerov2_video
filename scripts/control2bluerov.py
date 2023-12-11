#!/usr/bin/env python

import rospy
import pygame
from sensor_msgs.msg import Joy, Imu, FluidPressure, BatteryState
from std_msgs.msg import String, Bool, UInt16
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
import math
import time
import os
import datetime
import matplotlib.pyplot as plt

# Crear los mensajes
msgArm = Bool()
msgDeph = UInt16()
msgLateral = UInt16()
msgForward = UInt16()
msgYaw = UInt16()
msgRoll = UInt16()
msgPitch = UInt16()
msgModeSet = String()

# Crear los publicadores
pubArm = rospy.Publisher('/BlueRov2/arm', Bool, queue_size=10)
pubDeph = rospy.Publisher('BlueRov2/rc_channel3/set_pwm', UInt16, queue_size=1)
pubLateral = rospy.Publisher('BlueRov2/rc_channel6/set_pwm', UInt16, queue_size=1)
pubForward = rospy.Publisher('BlueRov2/rc_channel5/set_pwm', UInt16, queue_size=1)
pubYaw = rospy.Publisher('BlueRov2/rc_channel4/set_pwm', UInt16, queue_size=1)
pubRoll = rospy.Publisher('BlueRov2/rc_channel2/set_pwm', UInt16, queue_size=1)
pubPitch = rospy.Publisher('BlueRov2/rc_channel1/set_pwm', UInt16, queue_size=1)
pubModeSet = rospy.Publisher('/BlueRov2/mode/set', String, queue_size=10)

imuOrientX = imuOrientY = imuOrientZ = imuOrientW = 0.0
imuVelX = imuVelY = imuVelZ = 0.0
imuLinAccX = imuLinAccY = imuLinAccZ = 0.0

odomPosePosX = odomPosePosY = odomPosePosZ = 0.0
odomPoseOriX = odomPoseOriY = odomPoseOriZ = odomPoseOriW = 0.0
odomTwistLinX = odomTwistLinY = odomTwistLinZ = 0.0

msgBatteryVoltage = msgBatteryCurrent = msgBatteryPercentage = 0.0

msgFluidPress = 0.0
msgVariance = 0.0

errorOld = 0.0
errosYaw = 0.0

# yawDes = math.sin(t)
kpYaw = 2.5 
kdYaw = 1.5
T = 0.0

posrollrad = pospitchrad = posyawrad = 0.0

# Crear una figura y un eje
fig, ax = plt.subplots()

def rad_to_deg(rad):
    return rad * (180.0 / math.pi)


def write_to_file(filename, *args):
    # Get the current date and time
    now = datetime.datetime.now()
    
    # Format the date and time as a string
    timestamp = now.strftime("%Y-%m-%d_%H-%M")
    
    # Add the date and time to the filename, and add the .txt extension
    # filename = f"{filename}_{timestamp}.txt" # para python 3.6
    filename = "{}_{}.txt".format(filename, timestamp) # para python 2.7
    
    # Check if file exists, if not, create it
    if not os.path.exists(filename):
        open(filename, 'w').close()
    
    # Open the file in append mode
    with open(filename, 'a') as f:
        # Iterate over each argument
        for data in args:
            # Write the data followed by a newline
            f.write(str(data)+' ')
        f.write('\n')


def imuCallback(msg):
    global imuOrientX, imuOrientY, imuOrientZ, imuOrientW
    global imuVelX, imuVelY, imuVelZ
    global imuLinAccX, imuLinAccY, imuLinAccZ

    imuOrientX = msg.orientation.x
    imuOrientY = msg.orientation.y
    imuOrientZ = msg.orientation.z
    imuOrientW = msg.orientation.w

    imuVelX = msg.angular_velocity.x
    imuVelY = msg.angular_velocity.y
    imuVelZ = msg.angular_velocity.z

    imuLinAccX = msg.linear_acceleration.x
    imuLinAccY = msg.linear_acceleration.y
    imuLinAccZ = msg.linear_acceleration.z

# Funcion para manejar los mensajes de odometria recibidos
def odomCallback(msg):
    global odomPosePosx, odomPosePosy, odomPosePosz
    global odomPoseOrix, odomPoseOriy, odomPoseOriz, odomPoseOriw
    global odomTwistLinX, odomTwistLinY, odomTwistLinZ

    # Extraer la informacion de posicion del mensaje de Odometria
    odomPosePosX = msg.pose.pose.position.x
    odomPosePosY = msg.pose.pose.position.y
    odomPosePosZ = msg.pose.pose.position.z

    odomPoseOriX = msg.pose.pose.orientation.x
    odomPoseOriY = msg.pose.pose.orientation.y
    odomPoseOriZ = msg.pose.pose.orientation.z
    odomPoseOriW = msg.pose.pose.orientation.w

    odomTwistLinX = msg.twist.twist.linear.x
    odomTwistLinY = msg.twist.twist.linear.y
    odomTwistLinZ = msg.twist.twist.linear.z

# Funcion para manejar los mensajes de presion de fluidos recibidos
def presCallback(msg):
    global zbf_a, fluid_press

    msgFluidPress = msg.fluid_pressure
    msgVariance = msg.variance

def batteryCallback(msg):
    global msgBatteryVoltage, msgBatteryCurrent, msgBatteryPercentage

    msgBatteryVoltage = msg.voltage
    msgBatteryCurrent = msg.current
    msgBatteryPercentage = msg.percentage

def stopBluerov2():
    global msgArm, msgDeph, msgLateral, msgForward, msgYaw, msgRoll, msgPitch, msgModeSet
    global pubArm, pubDeph, pubLateral, pubForward, pubYaw, pubRoll, pubPitch, pubModeSet

    msgModeSet.data = "manual"
    msgArm.data = False
    msgDeph.data = 1500
    msgLateral.data = 1500
    msgForward.data = 1500
    msgYaw.data = 1500
    msgRoll.data = 1500
    msgPitch.data = 1500

    pubArm.publish(msgArm)
    pubLateral.publish(msgLateral)
    pubDeph.publish(msgDeph)
    pubForward.publish(msgForward)
    pubYaw.publish(msgYaw)
    pubRoll.publish(msgRoll)
    pubPitch.publish(msgPitch)
    pubModeSet.publish(msgModeSet)

def armBluerov2():
    global msgArm, pubArm

    msgArm.data = True
    pubArm.publish(msgArm)

def controlPDYaw(kp, kd, yawP, yawD, t):
    global errorOld, errosYaw

    errosYaw = yawD-yawP
    P = kp * errosYaw
    D = kd * (errosYaw - errorOld)/t
    u = P + D
    uThrusters = 1500 + u
    errorOld = errosYaw

    return uThrusters

def calculatePose(x, y, z, w):
    quaternion = (x, y, z, w)
    poseRollRad, posePitchRad, poseYawRad = euler_from_quaternion(quaternion)

    poseRollDegree = math.degrees(poseRollRad)
    posePitchDegree = math.degrees(posePitchRad)
    poseYawDegree = math.degrees(poseYawRad)

    return poseRollDegree, posePitchDegree, poseYawDegree

def manualControl(forward, lateral, yaw, deph, roll, pitch):
    global pubDeph, pubLateral, pubForward, pubYaw, pubRoll, pubPitch

    pubForward.publish(forward);
    pubLateral.publish(lateral);
    pubYaw.publish(yaw);
    pubDeph.publish(deph);
    pubRoll.publish(roll);
    pubPitch.publish(pitch);  

def trajectoryControl(t):
    # yawDesired = math.degrees(2*math.sin(0.25*T)*math.cos(0.06*T)-0.5)
    # yawDesired = math.degrees(2*math.sin(0.5*T))
    # yawDesired = math.degrees(1.5*math.sin(0.5*T))
    yawDesired = math.degrees(2*math.sin(0.5*t)*math.cos(0.125*t)-0.5)

    return yawDesired

def main():
    t = 0.0
    integracionTime = 0.05

    # Inicializar el nodo de ROS
    rospy.init_node('control2bluerov', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(20) # 10hz

    # Inicializa todos los módulos importados de Pygame
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Crear los suscriptores
    subPos = rospy.Subscriber("/BlueRov2/imu/data", Imu, imuCallback)  
    armed = False
    control = False
    while not rospy.is_shutdown(): 
        pygame.event.pump()
        joy_msg = Joy()
        T = t + integracionTime
        t = T

        yawDesired = trajectoryControl(T)
        Roll, Pitch, Yaw = calculatePose(imuOrientX, imuOrientY, imuOrientZ, imuOrientW)

        for i in range(joystick.get_numaxes()): 
            joy_msg.axes.append(joystick.get_axis(i))

        for i in range(joystick.get_numbuttons()):
            joy_msg.buttons.append(joystick.get_button(i))

        Xbutton = joy_msg.buttons[0]
        Abutton = joy_msg.buttons[1]
        Bbutton = joy_msg.buttons[2]
        Ybutton = joy_msg.buttons[3]

        msgLateral.data = joy_msg.axes[0]*100 + 1500
        msgForward.data = -joy_msg.axes[1]*100 + 1500
        msgYaw.data = joy_msg.axes[2]*100 + 1500
        msgDeph.data = joy_msg.axes[3]*100 + 1500
        # msgRoll.data = joy_msg.axes[4]*100 + 1500
        # msgPitch.data = joy_msg.axes[5]*100 + 1500

        if Abutton == 1 or armed == True:
            armed = True
            print("A button pressed Bluerov2 Armed")
            armBluerov2()
            manualControl(msgForward, 
                          msgLateral, 
                          msgYaw, 
                          msgDeph, 
                          msgRoll, 
                          msgPitch);
            
            print("Yaw degrees: ",Yaw, 
                  "Yaw Deseada: ", yawDesired)
            
            write_to_file('manual', "Forward ", msgForward,
                          "Lateral ", msgLateral,
                          "Yaw ", msgYaw, 
                          "Deph ", msgDeph,
                          "Roll ", msgRoll,
                          "Pitch ", msgPitch)

        if Xbutton == 1:
            armed = False
            control = False
            print("X button pressed Bluerov2 Disarm")
            stopBluerov2()

        elif Bbutton == 1 or control == True:
            control = True
            print("B button pressed") 
            armBluerov2()
            output = controlPDYaw(kpYaw, kdYaw, Yaw, yawDesired, T)
            pubYaw.publish(output);

            # imprime en pantalla los valores de los angulos, el control y el error
            print("yaw: ", Yaw,
                  "yaw Deseada: ", yawDesired,
                  "Control:", output,
                  "Error: ", Yaw-yawDesired)
            
            # Guarda los datos en un archivo de texto
            write_to_file("control",  "yaw: ", Yaw,
                          "yaw Deseada: ", yawDesired, 
                          "Control:", output,
                          "Error: ", Yaw-yawDesired)
            
        elif Ybutton == 1:
            print("Y button pressed") 
            

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



































# def write_to_file(filename, *args):
    
#     # Check if file exists, if not, create it
#     if not os.path.exists(filename):
#         open(filename, 'w').close()
    
#     # Open the file in append mode
#     with open(filename, 'a') as f:
#         # Iterate over each argument
#         for data in args:
#             # Write the data followed by a newline
#             f.write(str(data)+' ')
#         f.write('\n')





