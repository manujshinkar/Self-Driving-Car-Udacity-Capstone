#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
STOP_DIST = 5.0
MAX_DECEL = 0.5

MAX_SPEED = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600. # m/s

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.curr_pose = None
        self.waypoints_msg = None
        self.traffic_light_waypoint = None


        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.curr_pose = msg.pose
        if self.waypoints is not None:
            self.publish_final_waypoints()

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        if self.waypoints is None:
            self.waypoints = waypoints.waypoints
            self.waypoints_msg = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def get_stopping_waypoints(self, waypoints, redlight_index):

        if len(waypoints) < 1:
            return []

        for index, wp in enumerate(waypoints):

            if index > redlight_index:
                vel = 0
            else:
                #dist = self.distance(waypoints, index, redlight_index)
                if redlight_index >= len(waypoints):
                    dist = self.distance(waypoints[index].pose.pose.position, waypoints[len(waypoints)-1].pose.pose.position)
                else:
                    dist = self.distance(waypoints[index].pose.pose.position, waypoints[redlight_index].pose.pose.position)
                dist = max(0, dist - STOP_DIST)                
                vel  = math.sqrt(2 * MAX_DECEL * dist) 
                if vel < 1.:
                    vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

        return waypoints

    def get_closest_waypoint_index(self, pose, waypoints):
        closest_len = 100000
        closest_waypoint = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        for index, waypoint in enumerate(self.waypoints):
            dist = dl(pose.position, waypoint.pose.pose.position)
            if (dist < closest_len):
                closest_len = dist
                closest_waypoint = index
                
        return closest_waypoint

    def publish_final_waypoints(self):
        if self.curr_pose is not None:
            closest_waypoint_index = self.get_closest_waypoint_index(self.curr_pose, self.waypoints)
            lookahead_waypoints = self.waypoints[closest_waypoint_index:closest_waypoint_index+LOOKAHEAD_WPS]


            if self.traffic_light_waypoint is None or self.traffic_light_waypoint < 0:
                for i in range(len(lookahead_waypoints) - 1):                
                    self.set_waypoint_velocity(lookahead_waypoints, i, 11.176)

            else:
                redlight_lookahead_index = max(0, self.traffic_light_waypoint - closest_waypoint_index)
                lookahead_waypoints = self.get_stopping_waypoints(lookahead_waypoints, redlight_lookahead_index)

            for index,wp in enumerate(lookahead_waypoints):
                if wp.twist.twist.linear.x > MAX_SPEED:
                    wp.twist.twist.linear.x = MAX_SPEED - 1
            

            lane = Lane()
            lane.header = self.waypoints_msg.header
            lane.waypoints = lookahead_waypoints


            self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
