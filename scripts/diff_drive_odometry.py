#! /usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import sin, cos
#from diff_drive.pose import Pose
from diff_drive import odometry
right = 0.0
left = 0.0
sequence=0
frameid=""
angx=0.0
angy=0.0
angz=0.0
linx=0.0
liny=0.0
linz=0.0


class OdometryNode:

    def __init__(self):
        self.odometry = odometry.Odometry()

    def main(self):
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        # self.imuPub = rospy.Publisher('imu_data', Imu, queue_size=10)
        #self.tfPub = TransformBroadcaster()

        rospy.init_node('diff_drive_odometry')
        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        rospy.Subscriber("lwheel", Int16, self.leftCallback)
        rospy.Subscriber("rwheel", Int16, self.rightCallback)
        # rospy.Subscriber("imu", Imu, self.imu)
        #rospy.Subscriber("initialpose", PoseWithCovarianceStamped,
        #                self.on_initial_pose)

        self.ticksPerMeter = int(rospy.get_param('~ticks_meter'))
        self.wheelSeparation = float(rospy.get_param('~base_width'))
        self.rate = float(rospy.get_param('~rate', 33.0))
        self.baseFrameID = rospy.get_param('~base_frame_id', 'base_link')
        self.odomFrameID = rospy.get_param('~odom_frame_id', 'odom')
        self.encoderMin = int(rospy.get_param('~encoder_min', -32768))
        self.encoderMax = int(rospy.get_param('~encoder_max', 32767))
        self.tfPub=False

        self.odometry.setWheelSeparation(self.wheelSeparation)
        self.odometry.setTicksPerMeter(self.ticksPerMeter)
        self.odometry.setEncoderRange(self.encoderMin, self.encoderMax)
        self.odometry.setTime(rospy.get_time())

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        # global sequence, frameid, angx, angy, angz, linx, liny, linz
        
        self.odometry.updatePose(rospy.get_time())
        now = rospy.get_rostime()
        pose = self.odometry.getPose()

        q = quaternion_from_euler(0, 0, pose.theta)
        # self.tfPub.sendTransform(
        #     (pose.x, pose.y, 0),
        #     (q[0], q[1], q[2], q[3]),
        #     now,
        #     self.baseFrameID,
        #     self.odomFrameID
        # )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odomFrameID
        odom.child_frame_id = self.baseFrameID
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = pose.xVel
        odom.twist.twist.angular.z = pose.thetaVel
        odom.pose.covariance = [.01, 0, 0, 0, 0, 0,
                                 0, .01, 0, 0, 0, 0,
                                 0, 0, .01, 0, 0, 0,
                                 0, 0, 0, .01, 0, 0,
                                 0, 0, 0, 0, .01, 0,
                                 0, 0, 0, 0, 0, .01]
        # imu=Imu()
        # imu.header.seq=sequence
        # imu.header.stamp=now
        # imu.header.frame_id=frameid
        # imu.angular_velocity.x=angx
        # imu.angular_velocity.y=angy
        # imu.angular_velocity.z=angz
        # imu.linear_acceleration.x=linx
        # imu.linear_acceleration.y=liny
        # imu.linear_acceleration.z=linz
        # imu.linear_acceleration_covariance=[.01, 0, 0,
        #                                       0, .01, 0,
        #                                       0, 0, .01]
        # imu.angular_velocity_covariance=[.01, 0, 0,
        #                                    0, .01, 0,
        #                                    0, 0, .01]
        
        
        self.odomPub.publish(odom)
        # self.imuPub.publish(imu)

    def on_initial_pose(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        pose = Pose()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        pose.theta = yaw

        rospy.loginfo('Setting initial pose to %s', pose)
        self.odometry.setPose(pose)

    def rightCallback(self, msg):
        global left
        left = left+msg.data 
        self.odometry.updateLeftWheel(left)
	

    def leftCallback(self, msg):
        global right
        right = right+msg.data
        self.odometry.updateRightWheel(right)

    # def imu(self, msg):
    #     global sequence, frameid, angx, angy, angz, linx, liny, linz
    #     #header
    #     sequence=msg.header.seq
    #     frameid=msg.header.frame_id
    #     #data
    #     angx=msg.angular_velocity.x
    #     angy=msg.angular_velocity.y
    #     angz=msg.angular_velocity.z
    #     linx=msg.linear_acceleration.x
    #     liny=msg.linear_acceleration.y
    #     linz=msg.linear_acceleration.z



if __name__ == '__main__':
    try:
        node = OdometryNode()
        node.main()
    except rospy.ROSInterruptException:
        pass