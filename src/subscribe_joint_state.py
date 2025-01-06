import rospy
from std_msgs.msg import Float64MultiArray

def joint_status_callback(data):

    print('joint positions:', data.data)
    print('\n')

if __name__ == '__main__':

    rospy.init_node('subscribe_joint_state', anonymous=True)
    rospy.Subscriber('joint_angles', Float64MultiArray, joint_status_callback)
    rospy.spin()

