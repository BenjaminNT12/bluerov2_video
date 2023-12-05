#!/usr/bin/env python

import rospy
import pygame
from sensor_msgs.msg import Joy, Imu, FluidPressure
from std_msgs.msg import String, Bool, UInt16
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import math


# Crear los mensajes
arm_msg = Bool()
vertical = UInt16()
lateral = UInt16()
fforward = UInt16()
yaw = UInt16()
roll = UInt16()
pitch = UInt16()


# Crear los publicadores
pubArmar = rospy.Publisher('/BlueRov2/arm', Bool, queue_size=10)
pubVertical = rospy.Publisher('BlueRov2/rc_channel3/set_pwm', UInt16, queue_size=1)
pubLateral = rospy.Publisher('BlueRov2/rc_channel6/set_pwm', UInt16, queue_size=1)
pubForward = rospy.Publisher('BlueRov2/rc_channel5/set_pwm', UInt16, queue_size=1)
pubYaw = rospy.Publisher('BlueRov2/rc_channel4/set_pwm', UInt16, queue_size=1)
pubRoll = rospy.Publisher('BlueRov2/rc_channel2/set_pwm', UInt16, queue_size=1)
pubPitch = rospy.Publisher('BlueRov2/rc_channel1/set_pwm', UInt16, queue_size=1)



def posCallback(msg):
    # Extraer los componentes de la orientación del mensaje
    posax = msg.orientation.x
    posay = msg.orientation.y
    posaz = msg.orientation.z
    posaw = msg.orientation.w

    # Extraer los componentes de la velocidad angular del mensaje
    rollVelocity = msg.angular_velocity.x
    pitchVelocity = msg.angular_velocity.y
    yawVelocity = msg.angular_velocity.z

    # Crear un cuaternión a partir de los componentes de la orientación
    quaternion = (posax, posay, posaz, posaw)

    # Convertir el cuaternión a ángulos de Euler (roll, pitch, yaw)
    posrollrad, pospitchrad, posyawrad = euler_from_quaternion(quaternion)

    # Convertir los ángulos de radianes a grados
    posroll = math.degrees(posrollrad)
    pospitch = -1 * math.degrees(pospitchrad)
    posyaw = math.degrees(posyawrad)

    # Si el ángulo de yaw es positivo, restarle 360
    if posyaw > 0:
        posyaw = posyaw - 360



# Función para manejar los mensajes de odometría recibidos
def odomCallback(msg):
    # Extraer la información de posición del mensaje de Odometría
    posx = msg.pose.pose.position.x
    posy = msg.pose.pose.position.y
    posz = msg.pose.pose.position.z


# Inicializar zbf_a
zbf_a = 0

# Función para manejar los mensajes de presión de fluidos recibidos
def presCallback(msg):
    global zbf_a

    # Extraer la presión del fluido y la varianza del mensaje
    fluid_press = msg.fluid_pressure
    diff_press = msg.variance

    # Calcular z_baro a partir de la presión del fluido
    z_baro = fluid_press - 802.6

    # Calcular el coeficiente de filtrado a_z
    a_z = 0.1061

    # Calcular la z filtrada
    zbf = a_z * z_baro + (1 - a_z) * zbf_a

    # Actualizar zbf_a para la próxima iteración
    zbf_a = zbf


# Subscriptores
subPos = rospy.Subscriber("/BlueRov2/imu/data", Imu, posCallback)  # Topico real
odomPos = rospy.Subscriber("/BlueRov2/odometry", Odometry, odomCallback)  # Topico real

def talker():

    # Inicializar el nodo de ROS
    rospy.init_node('logitech controller') 
    pub = rospy.Publisher('joy', Joy, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # Inicializar pygame
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while not rospy.is_shutdown(): 
        pygame.event.pump()

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

        # Subscriptores

        # Imprimir los botones presionados y los valores de los joysticks
        #print("Buttons pressed: ", joy_msg.buttons)
        #print("Joystick values: ", joy_msg.axes)

        if Abutton == 1:
            print("A button pressed")

            # Establecer el valor del mensaje en True para armar el ROV
            arm_msg.data = True

            # Publicar el mensaje 
            pubArmar.publish(arm_msg)

        elif Xbutton == 1:
            print("X button pressed") 

            # Establecer el valor del mensaje en True para armar el ROV
            arm_msg.data = False

            # Publicar el mensaje
            pubArmar.publish(arm_msg)

        elif Bbutton == 1:
            print("B button pressed") 

            # Asignar valores a los datos de los mensajes
            arm_msg.data = False
            vertical.data = 1500
            lateral.data = 1500
            fforward.data = 1500
            yaw.data = 1500
            roll.data = 1500
            pitch.data = 1500

            # Publicar los mensajes
            pubVertical.publish(vertical)
            pubLateral.publish(lateral)
            pubForward.publish(fforward)
            pubYaw.publish(yaw)
            pubRoll.publish(roll)
            pubPitch.publish(pitch)
            pubArmar.publish(arm_msg)

        elif Ybutton == 1:
            print("Y button pressed") 

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