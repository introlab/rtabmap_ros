#!/usr/bin/env python

# Similar to map_assembler node, this minimal python example shows how
# to reconstruct the obstacle map by subscribing only to
# graph and latest data added to map (for constant network bandwidth usage).

import rospy
from sets import Set

import message_filters
from rtabmap_msgs.msg import MapGraph
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


posesDict = {}
cloudsDict = {}
assembledCloud = PointCloud2()
pub = rospy.Publisher('assembled_local_grids', PointCloud2, queue_size=10)

def callback(graph, cloud):
    global assembledCloud
    global posesDict
    global cloudsDict
    global pub

    begin = rospy.get_time()

    nodeId = graph.posesId[-1]
    pose = graph.poses[-1]
    size = cloud.width

    posesDict[nodeId] = pose
    cloudsDict[nodeId] = cloud

    # Update pose of our buffered clouds.
    # Check also if the clouds have moved because of a loop closure. If so, we have to update the rendering.
    maxDiff = 0
    for i in range(0,len(graph.posesId)):
        if graph.posesId[i] in posesDict:
            currentPose = posesDict[graph.posesId[i]].position
            newPose = graph.poses[i].position
            diff = max([abs(currentPose.x-newPose.x), abs(currentPose.y-newPose.y), abs(currentPose.z-newPose.z)])
            if maxDiff < diff:
                maxDiff = diff
        else:
            rospy.loginfo("Old node %d not found in cache, creating an empty cloud.", graph.posesId[i])
            posesDict[graph.posesId[i]] = graph.poses[i]
            cloudsDict[graph.posesId[i]] = PointCloud2()

    # If we don't move, some nodes would be removed from the graph, so remove them from our buffered clouds.
    newGraph = Set(graph.posesId)
    totalPoints = 0
    for p in posesDict.keys():
        if p not in newGraph:
            posesDict.pop(p)
            cloudsDict.pop(p)
        else:
            totalPoints = totalPoints + cloudsDict[p].width

    if maxDiff > 0.1:
        # if any node moved more than 10 cm, request an update of the assembled map so far
        newAssembledCloud = PointCloud2()
        rospy.loginfo("Map has been optimized! maxDiff=%.3fm, re-updating the whole map...", maxDiff)
        for i in range(0,len(graph.posesId)):
            posesDict[graph.posesId[i]] = graph.poses[i]
            t = TransformStamped()
            p = posesDict[graph.posesId[i]]
            t.transform.translation = p.position
            t.transform.rotation = p.orientation
            transformedCloud = do_transform_cloud(cloudsDict[graph.posesId[i]], t)
            if i==0:
                newAssembledCloud = transformedCloud
            else:
                newAssembledCloud.data = newAssembledCloud.data + transformedCloud.data
                newAssembledCloud.width = newAssembledCloud.width + transformedCloud.width
                newAssembledCloud.row_step = newAssembledCloud.row_step + transformedCloud.row_step
        assembledCloud = newAssembledCloud
    else:
        t = TransformStamped()
        t.transform.translation = pose.position
        t.transform.rotation = pose.orientation
        transformedCloud = do_transform_cloud(cloud, t)
        # just concatenate new cloud to current assembled map
        if assembledCloud.width == 0:
            assembledCloud = transformedCloud
        else:
            # Adding only the difference would be more efficient
            assembledCloud.data = assembledCloud.data + transformedCloud.data
            assembledCloud.width = assembledCloud.width + transformedCloud.width
            assembledCloud.row_step = assembledCloud.row_step + transformedCloud.row_step

    updateTime = rospy.get_time() - begin

    rospy.loginfo("Received node %d (%d pts) at xyz=%.2f %.2f %.2f, q_xyzw=%.2f %.2f %.2f %.2f (Map: Nodes=%d Points=%d Assembled=%d Update=%.0fms)", 
        nodeId, size,
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        len(cloudsDict), totalPoints, assembledCloud.width, updateTime*1000)

    assembledCloud.header = graph.header
    pub.publish(assembledCloud)

def main():
    rospy.init_node('assemble_local_grids', anonymous=True)
    graph_sub = message_filters.Subscriber('rtabmap/mapGraph', MapGraph)
    cloud_sub = message_filters.Subscriber('rtabmap/local_grid_obstacle', PointCloud2)

    ts = message_filters.TimeSynchronizer([graph_sub, cloud_sub], 2)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
