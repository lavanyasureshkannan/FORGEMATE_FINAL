#!/usr/bin/env python
import rospy
import time

from threading import Thread
from std_msgs.msg import Float64

def get_publishers():
    topic_name = '/forgemate/joint__position_controller/command'
    publishers = []
    for i in range(7):
        topic_name = '/forgemate/joint_' + str(i+1) + '_position_controller/command'
        publisher = rospy.Publisher(topic_name, Float64, queue_size=10)
        publishers.append(publisher)
    
    return publishers


def main():
    
    trajectory = []
    # angles_t0 = [0.3] * 7
    # angles_t1 = [0.9] * 7
    # angles_t2 = [0.5] * 7

    angles_t0 = [0.1, 0.1, 0.0, 3.0, -1.0, 0.0, 0.0]
    angles_t1 = [0.1, 0.1, 0.0, 3.0, -1.0, 0.2, 0.2]
    angles_t2 = [0.1, 0.3, 0.5, 3.0, -1.0, 0.2, 0.2]
    angles_t3 = [0.9, 0.3, 0.5, 3.0, -1.0, 0.2, 0.2]
    angles_t4 = [0.9, 0.1, 0.0, 3.0, -1.0, 0.2, 0.2]
    angles_t5 = [0.9, 0.1, 0.0, 3.0, -1.0, 0.0, 0.0]
    angles_t6 = [0.9, 0.3, 0.5, 3.0, -1.0, 0.0, 0.0]
    angles_t7 = [0.1, 0.3, 0.5, 3.0, -1.0, 0.0, 0.0]

    trajectory.append(angles_t0)
    trajectory.append(angles_t1)
    trajectory.append(angles_t2)
    trajectory.append(angles_t3)
    trajectory.append(angles_t4)
    trajectory.append(angles_t5)
    trajectory.append(angles_t6)
    trajectory.append(angles_t7)
    
    
    publishers = get_publishers()
    
    # for t in range(len(trajectory)):
    #     angles = trajectory[t]
    #     for i in range(len(angles)):
    #         rospy.loginfo('Angle = {} for joint = {}'.format(angles[i], i+1))
    #         publishers[i].publish(angles[i])
        
    #     rospy.sleep(2.0)

    for t in range(len(trajectory)):
        angles = trajectory[t]
        t1 = Thread(publishers[0].publish(angles[0]))
        t2 = Thread(publishers[1].publish(angles[1]))
        t3 = Thread(publishers[2].publish(angles[2]))
        t4 = Thread(publishers[3].publish(angles[3]))
        t5 = Thread(publishers[4].publish(angles[4]))
        t6 = Thread(publishers[5].publish(angles[5]))
        t7 = Thread(publishers[6].publish(angles[6]))
        
        t1.start()
        t2.start()
        t3.start()
        t4.start()
        t5.start()
        t6.start()
        t7.start()
        rospy.sleep(2.0)





if __name__=="__main__":
    rospy.init_node('forgemate_node', anonymous=True)
    rospy.loginfo('Initialized node...')
    while not rospy.is_shutdown():
        main()