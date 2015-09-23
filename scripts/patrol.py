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
        rospy.loginfo(rospy.get_caller_id() + "Goal %d reached! Publishing next goal in 1 sec...", int(waypoints[currentIndex]))
        currentIndex = (currentIndex+1) % len(waypoints)
    else:
        rospy.loginfo(rospy.get_caller_id() + "Goal %d failed! Retrying in 1 sec...", int(waypoints[currentIndex]))

    rospy.sleep(1.)

    msg = Goal()
    msg.node_id = int(waypoints[currentIndex])
    msg.node_label = ""
    msg.header.stamp = rospy.get_rostime()
    rospy.loginfo("Publishing goal %d! (%d/%d)", msg.node_id, currentIndex+1, len(waypoints))
    pub.publish(msg)

def main():
    rospy.init_node('patrol', anonymous=True)
    rospy.Subscriber("/rtabmap/goal_reached", Bool, callback)
    rospy.sleep(1.) # make sure that subscribers have seen this node before sending a goal

    # send the first goal
    msg = Goal()
    msg.node_id = int(waypoints[currentIndex])
    msg.node_label = ""
    while rospy.Time.now().secs == 0:
         rospy.loginfo("Waiting clock...")
         rospy.sleep(.1)
    msg.header.stamp = rospy.Time.now()
    rospy.loginfo("Publishing goal %d! (%d/%d)", msg.node_id, currentIndex+1, len(waypoints))
    pub.publish(msg)
    rospy.spin()    

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("usage: patrol.py waypointA waypointB waypointC ... (at least 2 waypoints)")
    else:
        waypoints = sys.argv[1:] 
        rospy.loginfo("Waypoints: [%s]", str(waypoints).strip('[]'))
        main()
