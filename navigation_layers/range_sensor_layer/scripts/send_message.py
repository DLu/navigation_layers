import rospy
from sensor_msgs.msg import Range

rospy.init_node('sender')
r = Range()

r.header.frame_id = '/base_laser_link'
r.field_of_view = 25*3.14/180
r.max_range = 100
r.range = 1

pub = rospy.Publisher('/sonar', Range)

while not rospy.is_shutdown():
    r.header.stamp = rospy.Time.now()
    pub.publish(r)

    rospy.sleep(5)
