#!/usr/bin/env python3
# CPU_temp node
#
# This node reads the current CPU Temperature (and ACPI temperatures
# on the mainboard) and outputs them every 5s to the topics:
#
# /CPU_temp
# /CPU_percent
# /MEM_percent
#
# uses the python library psutil
#
# Wayne Glettig, 26.01.2022

import rospy
from std_msgs.msg import Float32
import psutil

def cpu_temp_node():
    pub_CPU_temp  = rospy.Publisher('CPU_temp',      Float32, queue_size=10)
    pub_CPU_percent = rospy.Publisher('CPU_percent', Float32, queue_size=10)
    pub_MEM_percent = rospy.Publisher('MEM_percent', Float32, queue_size=10)
    
    rospy.init_node('CPU_temp')
    rate = rospy.Rate(1) # 1 message every 1 seconds
    rospy.loginfo(rospy.get_caller_id() + "  CPU_temp node launched. Publishing on /CPU_temp /CPU_percent /MEM_percent every 1s.")

    while not rospy.is_shutdown():
        CPU_temp = psutil.sensors_temperatures()['coretemp'][0].current
        CPU_percent = psutil.cpu_percent()
        MEM_percent = psutil.virtual_memory().percent
        
        pub_CPU_temp.publish(float(CPU_temp))
        pub_CPU_percent.publish(float(CPU_percent))
        pub_MEM_percent.publish(float(MEM_percent))
        rate.sleep()

if __name__ == '__main__':
    try:
        cpu_temp_node()
    except rospy.ROSInterruptException:
        pass

