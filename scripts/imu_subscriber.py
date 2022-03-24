#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from sensor_msgs.msg import Imu

def quat_to_angle(quat):
  rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
  return map(normalize_angle,rot.GetRPY())

def normalize_angle(angle):
    res = angle
    while res > math.pi:
        res -= 2.0*math.pi
    while res < -math.pi:
        res += 2.0*math.pi
    return res

def callback(data):
    #rpy
#    print(round(quat_to_angle(data.orientation),5), round(data.angular_velocity,5), round(data.linear_acceleration,3))
    print("Orientacion: ",quat_to_angle(data.orientation),"\n", "Velocidad Angular: ", data.angular_velocity,"\n", "Aceleracion lineal: ", data.linear_acceleration,"\n")
    #         The parameter received .DATA is the data of the communication by default to remove Data.Data data in such deforback (data).

def getangle(orientation):
    x=orientation.x
    y=orientation.y
    z=orientation.z
    w=orientation.w

    f=2*(w*y-z*z)

    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p=0
    if(-1<=f<=1):
        p = math.asin(f)
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    angleR = r*180/math.pi
    angleP = p*180/math.pi
    angleY = y*180/math.pi

    return {"angleR":angleR,"angleP":angleP,"angleY":angleY}


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

         # Start the node to name the node, if anoymous is a true node, the node will automatically add name, the actual name is represented by Listener_322345.
         # If you are fake, the system will not add name and use the user name. But only one of the same name nodes at a time, if there is one of the same Listener later
         #Ning the node of the name, the back node starts to log out the same node name in front.

    rospy.Subscriber("/BlueRov2/imu/data",Imu, callback)
    #rospy.Subscriber("robot_pose",Pose, callback)


         #  ,, subscription topic, and standard string format, call the callback function, call the function when there is data, remove the data

    # spin() simply keeps python from exiting until this node is stopped

         #
    rospy.spin()

if __name__ == '__main__':
    listener()
 # Function is not used as a module call
