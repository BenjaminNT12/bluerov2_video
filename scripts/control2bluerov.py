#!/usr/bin/env python

import rospy
import pygame
from sensor_msgs.msg import Joy, Imu, FluidPressure, BatteryState
from std_msgs.msg import String, Bool, UInt16
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
import math


# Crear los mensajes
arm_msg = Bool()
vertical = UInt16()
lateral = UInt16()
fforward = UInt16()
yaw = UInt16()
roll = UInt16()
pitch = UInt16()
mode = String()

# Crear los publicadores
pubArmar = rospy.Publisher('/BlueRov2/arm', Bool, queue_size=10)
pubVertical = rospy.Publisher('BlueRov2/rc_channel3/set_pwm', UInt16, queue_size=1)
pubLateral = rospy.Publisher('BlueRov2/rc_channel6/set_pwm', UInt16, queue_size=1)
pubForward = rospy.Publisher('BlueRov2/rc_channel5/set_pwm', UInt16, queue_size=1)
pubYaw = rospy.Publisher('BlueRov2/rc_channel4/set_pwm', UInt16, queue_size=1)
pubRoll = rospy.Publisher('BlueRov2/rc_channel2/set_pwm', UInt16, queue_size=1)
pubPitch = rospy.Publisher('BlueRov2/rc_channel1/set_pwm', UInt16, queue_size=1)
pubModeSet = rospy.Publisher('/BlueRov2/mode/set', String, queue_size=10)

orx = ory = orz = orw = 0.0
velX = velY = velZ = 0.0
linAccX = linAccY = linAccZ = 0.0

posroll = 0.0
pospitch = 0.0
posyaw = 0.0

zbf_a = 0.0
fluid_press = 0.0

posay = posaz = posaw = 0.0

posx = posy = posz = posax = 0.0
orix = oriy = oriz = oriw = 0.0
linx = liny = linz = 0.0

voltageBattery = currentBattery = percentageBattery = 0.0


def rad_to_deg(rad):
    return rad * (180.0 / math.pi)

def write_to_file(data):
    with open('output.txt', 'w') as f:
        f.write(str(data))

def posCallback(msg):
    global posroll, pospitch, posyaw
    global orx, ory, orz, orw
    global velX, velY, velZ
    global linAccX, linAccY, linAccZ
    # Extraer los componentes de la orientacion del mensaje
    orx = msg.orientation.x
    ory = msg.orientation.y
    orz = msg.orientation.z
    orw = msg.orientation.w

    # Extraer los componentes de la velocidad angular del mensaje
    velX = msg.angular_velocity.x
    velY = msg.angular_velocity.y
    velZ = msg.angular_velocity.z

    linAccX = msg.linear_acceleration.x
    linAccY = msg.linear_acceleration.y
    linAccZ = msg.linear_acceleration.z

    # Crear un cuaternion a partir de los componentes de la orientacion
    quaternion = (orx, ory, orz, orw)

    # Convertir el cuaternion a angulos de Euler (roll, pitch, yaw)
    posrollrad, pospitchrad, posyawrad = euler_from_quaternion(quaternion)

    # Convertir los angulos de radianes a grados
    posroll = math.degrees(posrollrad)
    pospitch = -1 * math.degrees(pospitchrad)
    posyaw = math.degrees(posyawrad)

    # Si el angulo de yaw es positivo, restarle 360
    if posyaw > 0:
        posyaw = posyaw - 360



# Funcion para manejar los mensajes de odometria recibidos
def odomCallback(msg):
    global posx, posy, posz
    global orix, oriy, oriz, oriw
    global linx, liny, linz
    # Extraer la informacion de posicion del mensaje de Odometria
    posx = msg.pose.pose.position.x
    posy = msg.pose.pose.position.y
    posz = msg.pose.pose.position.z

    orix = msg.pose.pose.orientation.x
    oriy = msg.pose.pose.orientation.y
    oriz = msg.pose.pose.orientation.z
    oriw = msg.pose.pose.orientation.w

    linx = msg.twist.twist.linear.x
    liny = msg.twist.twist.linear.y
    linz = msg.twist.twist.linear.z

# Funcion para manejar los mensajes de presion de fluidos recibidos
def presCallback(msg):
    global zbf_a, fluid_press

    # Extraer la presion del fluido y la varianza del mensaje
    fluid_press = msg.fluid_pressure
    diff_press = msg.variance

    # Calcular z_baro a partir de la presion del fluido
    z_baro = fluid_press - 802.6

    # Calcular el coeficiente de filtrado a_z
    a_z = 0.1061

    # Calcular la z filtrada
    zbf = a_z * z_baro + (1 - a_z) * zbf_a

    # Actualizar zbf_a para la proxima iteracion
    zbf_a = zbf


def batteryCallback(msg):
    global voltageBattery, currentBattery, percentageBattery

    voltageBattery = msg.voltage
    currentBattery = msg.current
    percentageBattery = msg.percentage
    pass

def talker():

    # Inicializar el nodo de ROS
    rospy.init_node('logitech_controller') 
    pub = rospy.Publisher('joy', Joy, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # Inicializar pygame
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while not rospy.is_shutdown(): 
        pygame.event.pump()
        # Subscriptores
        subPos = rospy.Subscriber("/BlueRov2/imu/data", Imu, posCallback)  # Topico real
        odomPos = rospy.Subscriber("/BlueRov2/odometry", Odometry, odomCallback)  # Topico real
        pres = rospy.Subscriber("/BlueRov2/pressure", FluidPressure, presCallback)
        subBattery = rospy.Subscriber("/BlueRov2/battery", BatteryState, batteryCallback)


        print("subPos: ",orx ,ory ,orz ,orw ,velX ,velY ,velZ ,linAccX ,linAccY ,linAccZ, posroll, pospitch, posyaw)
        print("odomPos: ", posx, posy, posz, orix, oriy, oriz, oriw, linx, liny, linz)
        print("Pressure: ", fluid_press)
        print("Battery: ", voltageBattery, currentBattery, percentageBattery)

        # Crear un mensaje Joy
        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()

        # Obtener los datos del joystick 
        for i in range(joystick.get_numaxes()): 
            joy_msg.axes.append(joystick.get_axis(i))

        for i in range(joystick.get_numbuttons()):
            joy_msg.buttons.append(joystick.get_button(i))

        Xbutton = joy_msg.buttons[0]
        Abutton = joy_msg.buttons[1]
        Bbutton = joy_msg.buttons[2]
        Ybutton = joy_msg.buttons[3]

        YawValue = joy_msg.axes[0]
        PitchValue = joy_msg.axes[1]
        RollValue = joy_msg.axes[2]
        ZValue = joy_msg.axes[3]

        # Manipular roll y pitch
        vertical.data = 1500 - (RollValue * 200) * 0.5
        lateral.data =  1500 + (YawValue * 200) * 0.5
        fforward.data = 1500 + (PitchValue * 200) * 0.5
        yaw.data = 1500 + (ZValue * 50)
        roll.data = 1500
        pitch.data = 1500

        # Imprimir los botones presionados y los valores de los joysticks
        #print("Buttons pressed: ", joy_msg.buttons)
        #print("Joystick values: ", joy_msg.axes)

        if Abutton == 1:
            print("A button pressed Bluerov2 Armed")

            # Establecer el valor del mensaje en True para armar el ROV
            arm_msg.data = True

            # Publicar el mensaje 
            pubArmar.publish(arm_msg)

        elif Xbutton == 1:
            print("X button pressed Bluerov2 Disarm")

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

        elif Bbutton == 1:
            print("B button pressed") 



        elif Ybutton == 1:
            print("Y button pressed") 
            # aqui es el control del bluerov2
            mode.data = "manual"
            pubModeSet.publish(mode)
        else:
            pubVertical.publish(vertical);
            pubLateral.publish(lateral);
            pubForward.publish(fforward);
            pubYaw.publish(yaw);
            pubRoll.publish(roll);
            pubPitch.publish(pitch);  

        # Publicar el mensaje 
        pub.publish(joy_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass