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
kpYaw = 1.5 
kdYaw = 0.5
T = 0.0

posrollrad = pospitchrad = posyawrad = 0.0

def rad_to_deg(rad):
    return rad * (180.0 / math.pi)

def write_to_file(filename, *args):
    
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
# def presCallback(msg):
#     global zbf_a, fluid_press

#     # Extraer la presion del fluido y la varianza del mensaje
#     fluid_press = msg.fluid_pressure
#     diff_press = msg.variance

#     # Calcular z_baro a partir de la presion del fluido
#     z_baro = fluid_press - 802.6

#     # Calcular el coeficiente de filtrado a_z
#     a_z = 0.1061

#     # Calcular la z filtrada
#     zbf = a_z * z_baro + (1 - a_z) * zbf_a

#     # Actualizar zbf_a para la proxima iteracion
#     zbf_a = zbf

def presCallback(msg):
    global depth

<<<<<<< HEAD
    # Extraer la presión del fluido del mensaje
    fluid_press = msg.fluid_pressure

    # Definir la presión atmosférica y la densidad del fluido (agua de mar)
    atmospheric_pressure = 101325  # en Pascales
    fluid_density = 1029  # en kg/m^3

    # Definir la aceleración de la gravedad
    g = 9.81  # en m/s^2

    # Calcular la profundidad
    depth = (fluid_press - atmospheric_pressure) / (fluid_density * g)

=======
    msgFluidPress = msg.fluid_pressure
    msgVariance = msg.variance
>>>>>>> cfdf3dd (control bluerov)

# def depthCallback(msg):
#     global depth

#     # Extraer la presión del fluido del mensaje
#     fluid_press = msg.fluid_pressure

#     # Definir la presión atmosférica en Pascales
#     atmospheric_pressure = 101325  # en Pascales

#     # Definir la constante de conversión de Pascal a metros de agua
#     pascal_to_m_water = 0.00010199773339984054

#     # Calcular la profundidad en metros de agua
#     depth = (fluid_press - atmospheric_pressure) * pascal_to_m_water

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

    if uThrusters > 1600:
        uThrusters = 1600
    elif uThrusters < 1400:
        uThrusters = 1400

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

def talker():
    t = 0.0
    integracionTime = 0.05

    rospy.init_node('control2bluerov', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(20) # 10hz
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    # Crear los suscriptores
    subPos = rospy.Subscriber("/BlueRov2/imu/data", Imu, imuCallback)  

    while not rospy.is_shutdown(): 
        pygame.event.pump()
        joy_msg = Joy()
        T = t + integracionTime
        t = T
        yawDesired = math.degrees(2*math.sin(T))*math.degrees(math.cos(0.25*T)-0.5)
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
        msgForward.data = joy_msg.axes[1]*100 + 1500
        msgYaw.data = joy_msg.axes[2]*100 + 1500
        msgDeph.data = joy_msg.axes[3]*100 + 1500
        # msgRoll.data = joy_msg.axes[4]*100 + 1500
        # msgPitch.data = joy_msg.axes[5]*100 + 1500

        if Abutton == 1:
<<<<<<< HEAD
            print("Bluerov2 Armed")

            # Establecer el valor del mensaje en True para armar el ROV
            arm_msg.data = True

            # Publicar el mensaje 
            pubArmar.publish(arm_msg)

        elif Xbutton == 1:
            print("Bluerov2 Disarm")

            # Asignar valores a los datos de los mensajes
            mode.data = "manual"
            arm_msg.data = False
            vertical.data = 1500
            lateral.data = 1500
            fforward.data = 1500
            yaw.data = 1500
            roll.data = 1500
            pitch.data = 1500

            # Publicar los mensajes
            pubArmar.publish(arm_msg)
            pubVertical.publish(vertical)
            pubLateral.publish(lateral)
            pubForward.publish(fforward)
            pubYaw.publish(yaw)
            pubRoll.publish(roll)
            pubPitch.publish(pitch)
            pubModeSet.publish(mode)
=======
            print("A button pressed Bluerov2 Armed")
            # armBluerov2()
            stopBluerov2()
            manualControl(msgForward, msgLateral, msgYaw, msgDeph, msgRoll, msgPitch);
            print("Forward: ", msgForward,
                  "Lateral: ", msgLateral,
                  "Yaw: ", msgYaw,
                  "Deph: ", msgDeph, 
                  "Roll: ", msgRoll,
                  "Pitch: ", msgPitch)
            write_to_file('manual.txt', "Forward ", msgForward,
                          "Lateral ", msgLateral,
                          "Yaw ", msgYaw, 
                          "Deph ", msgDeph,
                          "Roll ", msgRoll,
                          "Pitch ", msgPitch)

        elif Xbutton == 1:
            print("X button pressed Bluerov2 Disarm")
            stopBluerov2()
>>>>>>> cfdf3dd (control bluerov)

        elif Bbutton == 1:
            print("B button pressed") 
            armBluerov2()
            output = controlPDYaw(kpYaw, kdYaw, Yaw, yawDesired, T)
            pubYaw.publish(output);
            print("yaw: ", Yaw,
                  "yaw Deseada: ", yawDesired,
                  "Control:", output)
            
            write_to_file("control.txt", "yaw: ", Yaw,
                          "yaw Deseada: ", yawDesired, 
                          "Control:", output)
            
        elif Ybutton == 1:
            print("Y button pressed") 
            armBluerov2()
            output = controlPDYaw(kpYaw, kdYaw, Yaw, yawDesired, T)
            # pubYaw.publish(output);
            print("yaw: ", Yaw,
                  "yaw Deseada: ", yawDesired,
                  "Control:", output)
            
            write_to_file("control.txt", "yaw: ", Yaw,
                          "yaw Deseada: ", yawDesired, 
                          "Control:", output)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass