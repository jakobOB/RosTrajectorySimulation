import rospy
from std_msgs.msg import String



def graph_publisher():
    rospy.init_node('goal_publisher')
    

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # publish global path
        
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        graph_publisher()
    except rospy.ROSInterruptException:
        pass