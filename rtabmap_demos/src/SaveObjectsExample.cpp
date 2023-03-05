/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core/core.hpp>
#include <rtabmap_msgs/UserData.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap/core/Compression.h>
#include <visualization_msgs/MarkerArray.h>

class SaveObjectsExample
{
public:
	SaveObjectsExample() :
        objFramePrefix_("object"),
        frameId_("base_link")
    {
        ros::NodeHandle pnh("~");
        pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);
        pnh.param("frame_id", frameId_, frameId_);

        ros::NodeHandle nh;
        subObjects_ = nh.subscribe("objectsStamped", 1, &SaveObjectsExample::objectsDetectedCallback, this);
        subsMapData_ = nh.subscribe("mapData", 1, &SaveObjectsExample::mapDataCallback, this);
        pub_ = nh.advertise<rtabmap_msgs::UserData>("objectsData", 1);
        pubMarkers_ = nh.advertise<visualization_msgs::MarkerArray>("objectsMarkers", 1);
    }

    // from find_object_2d to rtabmap
    void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
    {
        if(msg->objects.data.size())
        {
            cv::Mat data(msg->objects.data.size()/12, 9, CV_64FC1);
            for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
            {
                // get data
                int id = (int)msg->objects.data[i];
                std::string objectFrameId = uFormat("%s_%d", objFramePrefix_.c_str(),id); // "object_1", "object_2"

                // get pose of the object in base frame
                tf::StampedTransform pose;
                try
                {
                    tfListener_.lookupTransform(frameId_, objectFrameId, msg->header.stamp, pose);
                }
                catch(tf::TransformException & ex)
                {
                    ROS_WARN("%s",ex.what());
                    return;
                }

                data.at<double>(i, 0) = double(id);
                data.at<double>(i, 1) = ros::Time(msg->header.stamp).toSec();
                data.at<double>(i, 2) = pose.getOrigin().x();
                data.at<double>(i, 3) = pose.getOrigin().y();
                data.at<double>(i, 4) = pose.getOrigin().z();
                data.at<double>(i, 5) = pose.getRotation().x();
                data.at<double>(i, 6) = pose.getRotation().y();
                data.at<double>(i, 7) = pose.getRotation().z();
                data.at<double>(i, 8) = pose.getRotation().w();
            }

            rtabmap_msgs::UserData dataMsg;
            dataMsg.header.frame_id = msg->header.frame_id;
            dataMsg.header.stamp = msg->header.stamp;
            rtabmap_conversions::userDataToROS(data, dataMsg, false);
            pub_.publish(dataMsg);
        }
    }

    // from rtabmap to rviz visualization
    void mapDataCallback(const rtabmap_msgs::MapDataConstPtr & msg)
    {
        std::map<int, rtabmap::Signature> signatures;
        std::map<int, rtabmap::Transform> poses;
        std::multimap<int, rtabmap::Link> links;
        rtabmap::Transform mapToOdom;
        rtabmap_conversions::mapDataFromROS(*msg, poses, links, signatures, mapToOdom);

        // handle the case where we can receive only latest data, or if all data are published
        for(std::map<int, rtabmap::Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
        {
        	int id = iter->first;
        	rtabmap::Signature & node = iter->second;

			nodeStamps_.insert(std::make_pair(node.getStamp(), node.id()));

			if(!node.sensorData().userDataCompressed().empty() && nodeToObjects_.find(id)==nodeToObjects_.end())
			{
				cv::Mat data = rtabmap::uncompressData(node.sensorData().userDataCompressed());
				ROS_ASSERT(data.cols == 9 && data.type() == CV_64FC1);
				ROS_INFO("Node %d has %d object(s)", id, data.rows);
				nodeToObjects_.insert(std::make_pair(id, data));
			}
        }

		// for the logic below, we should keep only stamps for
		// nodes still in the graph (in case nodes are ignored when not moving)
		std::map<double, int> nodeStamps;
		for(std::map<double, int>::iterator iter=nodeStamps_.begin(); iter!=nodeStamps_.end(); ++iter)
		{
			std::map<int, rtabmap::Transform>::const_iterator jter = poses.find(iter->second);
			if(jter != poses.end())
			{
				nodeStamps.insert(*iter);
			}
		}

        // Publish markers accordingly to current optimized graph
        std::map<int, float> objectsAdded;
        visualization_msgs::MarkerArray markers;

        if(nodeToObjects_.size())
        {
            for(std::map<int, rtabmap::Transform>::iterator iter=poses.lower_bound(1); iter!=poses.end(); ++iter)
            {
                if(nodeToObjects_.find(iter->first) != nodeToObjects_.end())
                {
                    cv::Mat data = nodeToObjects_.at(iter->first);
                    for(int i=0; i<data.rows; ++i)
                    {
                        int objId = int(data.at<double>(i,0));
                        double stamp = data.at<double>(i,1);

                        // The object detection may have been taken between two nodes, interpolate its position.
                        std::map<double, int>::iterator previousNode = nodeStamps.lower_bound(stamp); // lower bound of the stamp
                        if(previousNode!=nodeStamps.end() && previousNode->first > stamp && previousNode != nodeStamps.begin())
                        {
                            --previousNode;
                        }
                        std::map<double, int>::iterator nextNode = nodeStamps.upper_bound(stamp); // upper bound of the stamp

                        if(previousNode != nodeStamps.end() &&
                           nextNode != nodeStamps.end() &&
                           previousNode->second != nextNode->second &&
                           poses.find(previousNode->second)!=poses.end() && poses.find(nextNode->second)!=poses.end())
                        {
                            rtabmap::Transform poseA = poses.at(previousNode->second);
                            rtabmap::Transform poseB = poses.at(nextNode->second);
                            double stampA = previousNode->first;
                            double stampB = nextNode->first;
                            UASSERT(stamp>=stampA && stamp <=stampB);

                            double ratio = (stamp-stampA)/(stampB-stampA);
                            rtabmap::Transform robotPose = poseA.interpolate(ratio, poseB);

                            // transform object pose in map frame
                            rtabmap::Transform objPose(
                                    data.at<double>(i,2), data.at<double>(i,3), data.at<double>(i,4),
                                    data.at<double>(i,5), data.at<double>(i,6), data.at<double>(i,7), data.at<double>(i,8));
                            float distanceFromNode = objPose.getNorm();
                            objPose = robotPose * objPose;

                            if(objectsAdded.find(objId) == objectsAdded.end())
                            {
                                visualization_msgs::Marker marker;
                                marker.header.frame_id = msg->header.frame_id;
                                marker.header.stamp = msg->header.stamp;
                                marker.ns = "objects";
                                marker.id = objId;
                                marker.action = visualization_msgs::Marker::ADD;
                                marker.pose.position.x = objPose.x();
                                marker.pose.position.y = objPose.y();
                                marker.pose.position.z = objPose.z();
                                Eigen::Quaterniond q = objPose.getQuaterniond();
                                marker.pose.orientation.x = q.x();
                                marker.pose.orientation.y = q.y();
                                marker.pose.orientation.z = q.z();
                                marker.pose.orientation.w = q.w();
                                marker.scale.x = 0.3;
                                marker.scale.y = 0.3;
                                marker.scale.z = 0.3;
                                marker.color.a = 0.8;
                                marker.color.r = 0.0;
                                marker.color.g = 1.0;
                                marker.color.b = 0.0;

                                // cube
                                marker.type = visualization_msgs::Marker::CUBE;
                                markers.markers.push_back(marker);

                                // text
                                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                                marker.text = uFormat("%s_%d", objFramePrefix_.c_str(), objId);
                                marker.id = -objId;
                                marker.color.a = 1.0;
                                marker.pose.position.z += 0.3; // text over the cube
                                markers.markers.push_back(marker);

                                objectsAdded.insert(std::make_pair(objId, distanceFromNode));
                            }
                            else
                            {
                                // update pose of the marker only if current observation is closer to robot
                                if(distanceFromNode < objectsAdded.at(objId))
                                {
                                    for(unsigned int j=0; j<markers.markers.size(); ++j)
                                    {
                                        if(markers.markers[j].id == objId || markers.markers[j].id == -objId)
                                        {
                                            markers.markers[j].pose.position.x = objPose.x();
                                            markers.markers[j].pose.position.y = objPose.y();
                                            markers.markers[j].pose.position.z = objPose.z() + (markers.markers[j].id<0?0.3:0);
                                            Eigen::Quaterniond q = objPose.getQuaterniond();
                                            markers.markers[j].pose.orientation.x = q.x();
                                            markers.markers[j].pose.orientation.y = q.y();
                                            markers.markers[j].pose.orientation.z = q.z();
                                            markers.markers[j].pose.orientation.w = q.w();
                                        }
                                    }
                                    objectsAdded.at(objId) = distanceFromNode;
                                }
                            }
                        }
                    }
                }
            }
            if(!markers.markers.empty())
            {
                pubMarkers_.publish(markers);
            }
        }
    }

private:
    std::string objFramePrefix_;
    ros::Subscriber subObjects_;
    ros::Subscriber subsMapData_;
    tf::TransformListener tfListener_;
    ros::Publisher pub_;
    ros::Publisher pubMarkers_;
    std::map<int, cv::Mat> nodeToObjects_;
    std::map<double, int> nodeStamps_; // <stamp, id>
    std::string frameId_;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "save_objects_example");

    SaveObjectsExample sync;
    ros::spin();
}
