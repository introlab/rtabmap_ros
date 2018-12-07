#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Bool
from rtabmap_ros.msg import Goal

pub = rospy.Publisher('/rtabmap/goal_node', Goal, queue_size=1)
waypoints = []
currentIndex = 0
waitingTime = 1.0

def callback(data):
    global currentIndex
    global waitingTime
    if data.data:
        rospy.loginfo(rospy.get_caller_id() + ": Goal '%s' reached! Publishing next goal in %.1f sec...", waypoints[currentIndex], waitingTime)
    else:
        rospy.loginfo(rospy.get_caller_id() + ": Goal '%s' failed! Publishing next goal in %.1f sec...", waypoints[currentIndex], waitingTime)

    currentIndex = (currentIndex+1) % len(waypoints)

    # Waiting time before sending next goal
    rospy.sleep(waitingTime)

    msg = Goal()
    try:
        int(waypoints[currentIndex])
        is_dig = True
    except ValueError:
        is_dig = False
    if is_dig:
        msg.node_id = int(waypoints[currentIndex])
        msg.node_label = ""
    else:
        msg.node_id = 0
        msg.node_label = waypoints[currentIndex]
    
    rospy.loginfo(rospy.get_caller_id() + ": Publishing goal '%s'! (%d/%d)", waypoints[currentIndex], currentIndex+1, len(waypoints))
    msg.header.stamp = rospy.get_rostime()
    pub.publish(msg)

def main():
    rospy.init_node('patrol', anonymous=False)
    rospy.Subscriber("/rtabmap/goal_reached", Bool, callback)
    global waitingTime
    waitingTime = rospy.get_param('~time', waitingTime)
    rospy.sleep(1.) # make sure that subscribers have seen this node before sending a goal

    rospy.loginfo(rospy.get_caller_id() + ": Waypoints: [%s]", str(waypoints).strip('[]'))
    rospy.loginfo(rospy.get_caller_id() + ": time: %f", waitingTime)

    # send the first goal
    msg = Goal()
    try:
        int(waypoints[currentIndex])
        is_dig = True
    except ValueError:
        is_dig = False
    if is_dig:
        msg.node_id = int(waypoints[currentIndex])
        msg.node_label = ""
    else:
        msg.node_id = 0
        msg.node_label = waypoints[currentIndex]
    while rospy.Time.now().secs == 0:
         rospy.loginfo(rospy.get_caller_id() + ": Waiting clock...")
         rospy.sleep(.1)
    msg.header.stamp = rospy.Time.now()
    rospy.loginfo(rospy.get_caller_id() + ": Publishing goal '%s'! (%d/%d)", waypoints[currentIndex], currentIndex+1, len(waypoints))
    pub.publish(msg)
    rospy.spin()    

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("usage: patrol.py waypointA waypointB waypointC ... [_time:=1] [topic remaps] (at least 2 waypoints, can be node id, landmark or label)")
    else:
        waypoints = sys.argv[1:] 
        waypoints = [x for x in waypoints if not x.startswith('/') and not x.startswith('_')]
        main()
