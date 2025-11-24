#!/usr/bin/env python3
"""
ROS node that reshapes Gazebo IMU data to match the real LIMO driver output.

관성계는 Gazebo가 제공하는 orientation / angular velocity / linear acceleration을
그대로 사용하되, 실제 limo_driver처럼 roll/pitch를 0으로 두고 yaw만 유지한다.
이렇게 하면 시뮬레이터와 실제 로봇의 IMU 토픽 형식이 동일해진다.
"""

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ImuOrientationAdapter:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/imu_raw")
        self.output_topic = rospy.get_param("~output_topic", "/imu")
        self.frame_id = rospy.get_param("~frame_id", "imu_link")

        self.publisher = rospy.Publisher(self.output_topic, Imu, queue_size=10)
        rospy.Subscriber(self.input_topic, Imu, self.callback, queue_size=10)

        rospy.loginfo(
            "[imu_orientation_adapter] Bridging %s -> %s (frame: %s)",
            self.input_topic,
            self.output_topic,
            self.frame_id,
        )

    def callback(self, msg: Imu):
        quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        _, _, yaw = euler_from_quaternion(quat)
        yaw_only = quaternion_from_euler(0.0, 0.0, yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = msg.header.stamp
        imu_msg.header.frame_id = self.frame_id

        imu_msg.orientation.x = yaw_only[0]
        imu_msg.orientation.y = yaw_only[1]
        imu_msg.orientation.z = yaw_only[2]
        imu_msg.orientation.w = yaw_only[3]

        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.linear_acceleration = msg.linear_acceleration

        imu_msg.orientation_covariance[0] = 1e-6
        imu_msg.orientation_covariance[4] = 1e-6
        imu_msg.orientation_covariance[8] = 1e-6

        imu_msg.angular_velocity_covariance[0] = 1e-6
        imu_msg.angular_velocity_covariance[4] = 1e-6
        imu_msg.angular_velocity_covariance[8] = 1e-6

        imu_msg.linear_acceleration_covariance[0] = 1.0
        imu_msg.linear_acceleration_covariance[4] = 1.0
        imu_msg.linear_acceleration_covariance[8] = 1.0

        self.publisher.publish(imu_msg)


def main():
    rospy.init_node("imu_orientation_adapter")
    adapter = ImuOrientationAdapter()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


