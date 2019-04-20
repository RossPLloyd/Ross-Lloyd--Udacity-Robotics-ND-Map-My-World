#!/usr/bin/env python
from math import pi, cos, sin, copysign

import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
from roboclaw_node.msg import Motors_currents, Wheels_speeds
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
import threading

__author__ = "guillaumedoisy@gmail.com"


# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            rospy.logerr("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            rospy.logerr("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, cur_theta),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

class Node:
    def __init__(self):
        self.lock = threading.Lock()
        rospy.init_node("roboclaw_node",log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")

        self.wheels_speeds_pub = rospy.Publisher('/motors/commanded_speeds', Wheels_speeds, queue_size=1)
        self.motors_currents_pub = rospy.Publisher('/motors/read_currents', Motors_currents, queue_size=1)

        self.address = int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        # TODO need someway to check if address is correct


        self.MAX_ABS_LINEAR_SPEED = float(rospy.get_param("~max_abs_linear_speed", "1.0"))
        self.MAX_ABS_ANGULAR_SPEED = float(rospy.get_param("~max_abs_angular_speed", "1.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~ticks_per_meter", "4342.2"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.315"))
        self.ACC_LIM = float(rospy.get_param("~acc_lim", "0.1"))

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.last_set_speed_time = rospy.get_rostime()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)

        rospy.sleep(1)

        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_abs_linear_speed %f", self.MAX_ABS_LINEAR_SPEED)
        rospy.logdebug("max_abs_angular_speed %f", self.MAX_ABS_ANGULAR_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(30)
        while not rospy.is_shutdown():
            r_time.sleep()

    def cmd_vel_callback(self, twist):
        with self.lock:
            self.last_set_speed_time = rospy.get_rostime()

            linear_x = twist.linear.x
            angular_z = twist.angular.z
            if abs(linear_x) > self.MAX_ABS_LINEAR_SPEED:
                linear_x = copysign(self.MAX_ABS_LINEAR_SPEED, linear_x)
            if abs(angular_z) > self.MAX_ABS_ANGULAR_SPEED:
                angular_z = copysign(self.MAX_ABS_ANGULAR_SPEED, angular_z)

            vr = linear_x - angular_z * self.BASE_WIDTH / 2.0  # m/s
            vl = linear_x + angular_z * self.BASE_WIDTH / 2.0

            vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
            vl_ticks = int(vl * self.TICKS_PER_METER)

            v_wheels= Wheels_speeds()
            v_wheels.wheel1=vl
            v_wheels.wheel2=vr
            self.wheels_speeds_pub.publish(v_wheels)


            rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(roboclaw.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
