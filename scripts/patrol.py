#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Bool
from rtabmap_ros.msg import Goal

pub = rospy.Publisher('/rtabmap/goal_node', Goal, queue_size=1)
waypoints = []
currentIndex = 0

def callback(data):
    global currentIndex
    if data.data:
        rospy.loginfo(rospy.get_caller_id() + "Goal '%s' reached! Publishing next goal in 1 sec...", waypoints[currentIndex])
    else:
        rospy.loginfo(rospy.get_caller_id() + "Goal '%s' failed! Publishing next goal in 1 sec...", waypoints[currentIndex])

    currentIndex = (currentIndex+1) % len(waypoints)

    rospy.sleep(1.)

    msg = Goal()
    if waypoints[currentIndex].isdigit():
        msg.node_id = int(waypoints[currentIndex])
        msg.node_label = ""
    else:
        msg.node_id = 0
        msg.node_label = waypoints[currentIndex]
    msg.header.stamp = rospy.get_rostime()
    rospy.loginfo("Publishing goal '%s'! (%d/%d)", waypoints[currentIndex], currentIndex+1, len(waypoints))
    pub.publish(msg)

def main():
    rospy.init_node('patrol', anonymous=False)
    rospy.Subscriber("/rtabmap/goal_reached", Bool, callback)
    rospy.sleep(1.) # make sure that subscribers have seen this node before sending a goal

    # send the first goal
    msg = Goal()
    if waypoints[currentIndex].isdigit():
        msg.node_id = int(waypoints[currentIndex])
        msg.node_label = ""
    else:
        msg.node_id = 0
        msg.node_label = waypoints[currentIndex]
    while rospy.Time.now().secs == 0:
         rospy.loginfo("Waiting clock...")
         rospy.sleep(.1)
    msg.header.stamp = rospy.Time.now()
    rospy.loginfo("Publishing goal '%s'! (%d/%d)", waypoints[currentIndex], currentIndex+1, len(waypoints))
    pub.publish(msg)
    rospy.spin()    

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("usage: patrol.py waypointA waypointB waypointC ... (at least 2 waypoints, can be node id or label)")
    else:
        waypoints = sys.argv[1:] 
        rospy.loginfo("Waypoints: [%s]", str(waypoints).strip('[]'))
        main()
