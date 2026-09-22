#!/usr/bin/env python3
import rospy
import tf_conversions
import tf2_ros
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class OrientationBroadster:
    def __init__(self) -> None:
        self.imu_sub = rospy.Subscriber("/IMU_ICM20948", Imu, self.msg_callback, queue_size = 5)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        
    def msg_callback(self, msg: Imu):
        transform = tf2_ros.TransformStamped()

        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        transform.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("MeasTfBroadcaster")
    OrientationBroadster()
    rospy.spin()