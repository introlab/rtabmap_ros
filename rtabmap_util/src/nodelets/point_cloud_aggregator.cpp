/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/util3d_filtering.h>

namespace rtabmap_util
{

/**
 * Nodelet used to merge point clouds from different sensors into a single
 * assembled cloud. If fixed_frame_id is set and approx_sync is true,
 * the clouds are adjusted to include the displacement of the robot
 * in the output cloud.
 */
class PointCloudAggregator : public nodelet::Nodelet
{
public:
	PointCloudAggregator() :
		warningThread_(0),
		callbackCalled_(false),
		exactSync4_(0),
		approxSync4_(0),
		exactSync3_(0),
		approxSync3_(0),
		exactSync2_(0),
		approxSync2_(0),
		waitForTransformDuration_(0.1),
		xyzOutput_(false)
	{}

	virtual ~PointCloudAggregator()
	{
	    delete exactSync4_;
	    delete approxSync4_;
	    delete exactSync3_;
	    delete approxSync3_;
	    delete exactSync2_;
	    delete approxSync2_;

	    if(warningThread_)
		{
			callbackCalled_=true;
			warningThread_->join();
			delete warningThread_;
		}
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 1;
		int syncQueueSize = 5;
		int count = 2;
		bool approx=true;
		double approxSyncMaxInterval = 0.0;
		pnh.param("topic_queue_size", queueSize, queueSize);
		if(pnh.hasParam("queue_size") && !pnh.hasParam("sync_queue_size"))
		{
			pnh.param("queue_size", syncQueueSize, syncQueueSize);
			ROS_WARN("Parameter \"queue_size\" has been renamed "
					"to \"sync_queue_size\" and will be removed "
					"in future versions! The value (%d) is copied to "
					"\"sync_queue_size\".", syncQueueSize);
		}
		else
		{
			pnh.param("sync_queue_size", syncQueueSize, syncQueueSize);
		}
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("fixed_frame_id", fixedFrameId_, fixedFrameId_);
		pnh.param("approx_sync", approx, approx);
		pnh.param("approx_sync_max_interval", approxSyncMaxInterval, approxSyncMaxInterval);
		pnh.param("count", count, count);
		pnh.param("wait_for_transform_duration", waitForTransformDuration_, waitForTransformDuration_);
		pnh.param("xyz_output", xyzOutput_, xyzOutput_);

		cloudSub_1_.subscribe(nh, "cloud1", queueSize);
		cloudSub_2_.subscribe(nh, "cloud2", queueSize);

		std::string subscribedTopicsMsg;
		if(count == 4)
		{
			cloudSub_3_.subscribe(nh, "cloud3", queueSize);
			cloudSub_4_.subscribe(nh, "cloud4", queueSize);
			if(approx)
			{
				approxSync4_ = new message_filters::Synchronizer<ApproxSync4Policy>(ApproxSync4Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_, cloudSub_4_);
				if(approxSyncMaxInterval > 0.0)
					approxSync4_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
				approxSync4_->registerCallback(boost::bind(&rtabmap_util::PointCloudAggregator::clouds4_callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
			}
			else
			{
				exactSync4_ = new message_filters::Synchronizer<ExactSync4Policy>(ExactSync4Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_, cloudSub_4_);
				exactSync4_->registerCallback(boost::bind(&rtabmap_util::PointCloudAggregator::clouds4_callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
			}
			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s,\n   %s,\n   %s",
					getName().c_str(),
					approx?"approx":"exact",
					approx&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
					cloudSub_1_.getTopic().c_str(),
					cloudSub_2_.getTopic().c_str(),
					cloudSub_3_.getTopic().c_str(),
					cloudSub_4_.getTopic().c_str());
		}
		else if(count == 3)
		{
			cloudSub_3_.subscribe(nh, "cloud3", queueSize);
			if(approx)
			{
				approxSync3_ = new message_filters::Synchronizer<ApproxSync3Policy>(ApproxSync3Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_);
				if(approxSyncMaxInterval > 0.0)
					approxSync3_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
				approxSync3_->registerCallback(boost::bind(&rtabmap_util::PointCloudAggregator::clouds3_callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
			}
			else
			{
				exactSync3_ = new message_filters::Synchronizer<ExactSync3Policy>(ExactSync3Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_);
				exactSync3_->registerCallback(boost::bind(&rtabmap_util::PointCloudAggregator::clouds3_callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
			}
			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s,\n   %s",
					getName().c_str(),
					approx?"approx":"exact",
					approx&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
					cloudSub_1_.getTopic().c_str(),
					cloudSub_2_.getTopic().c_str(),
					cloudSub_3_.getTopic().c_str());
		}
		else
		{
			if(approx)
			{
				approxSync2_ = new message_filters::Synchronizer<ApproxSync2Policy>(ApproxSync2Policy(queueSize), cloudSub_1_, cloudSub_2_);
				if(approxSyncMaxInterval > 0.0)
					approxSync2_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
				approxSync2_->registerCallback(boost::bind(&rtabmap_util::PointCloudAggregator::clouds2_callback, this, boost::placeholders::_1, boost::placeholders::_2));
			}
			else
			{
				exactSync2_ = new message_filters::Synchronizer<ExactSync2Policy>(ExactSync2Policy(queueSize), cloudSub_1_, cloudSub_2_);
				exactSync2_->registerCallback(boost::bind(&rtabmap_util::PointCloudAggregator::clouds2_callback, this, boost::placeholders::_1, boost::placeholders::_2));
			}
			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s",
					getName().c_str(),
					approx?"approx":"exact",
					approx&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
					cloudSub_1_.getTopic().c_str(),
					cloudSub_2_.getTopic().c_str());
		}

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);

		warningThread_ = new boost::thread(boost::bind(&PointCloudAggregator::warningLoop, this, subscribedTopicsMsg, approx));
		NODELET_INFO("%s", subscribedTopicsMsg.c_str());
	}

	void clouds4_callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg_1,
						 const sensor_msgs::PointCloud2ConstPtr & cloudMsg_2,
						 const sensor_msgs::PointCloud2ConstPtr & cloudMsg_3,
						 const sensor_msgs::PointCloud2ConstPtr & cloudMsg_4)
	{
		std::vector<sensor_msgs::PointCloud2ConstPtr> clouds;
		clouds.push_back(cloudMsg_1);
		clouds.push_back(cloudMsg_2);
		clouds.push_back(cloudMsg_3);
		clouds.push_back(cloudMsg_4);

		combineClouds(clouds);
	}
	void clouds3_callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg_1,
	                     const sensor_msgs::PointCloud2ConstPtr & cloudMsg_2,
	                     const sensor_msgs::PointCloud2ConstPtr & cloudMsg_3)
	{
		std::vector<sensor_msgs::PointCloud2ConstPtr> clouds;
		clouds.push_back(cloudMsg_1);
		clouds.push_back(cloudMsg_2);
		clouds.push_back(cloudMsg_3);

		combineClouds(clouds);
	}
	void clouds2_callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg_1,
						 const sensor_msgs::PointCloud2ConstPtr & cloudMsg_2)
	{
		std::vector<sensor_msgs::PointCloud2ConstPtr> clouds;
		clouds.push_back(cloudMsg_1);
		clouds.push_back(cloudMsg_2);

		combineClouds(clouds);
	}
	void combineClouds(const std::vector<sensor_msgs::PointCloud2ConstPtr> & cloudMsgs)
	{
		callbackCalled_ = true;
		ROS_ASSERT(cloudMsgs.size() > 1);
		if(cloudPub_.getNumSubscribers())
		{
			pcl::PCLPointCloud2::Ptr output(new pcl::PCLPointCloud2);

			std::string frameId = frameId_;
			if(!frameId.empty() && frameId.compare(cloudMsgs[0]->header.frame_id) != 0)
			{
				sensor_msgs::PointCloud2 tmp;
				pcl_ros::transformPointCloud(frameId, *cloudMsgs[0], tmp, tfListener_);
				pcl_conversions::toPCL(tmp, *output);
			}
			else
			{
				pcl_conversions::toPCL(*cloudMsgs[0], *output);
				frameId = cloudMsgs[0]->header.frame_id;
			}

			if(xyzOutput_ && !output->data.empty())
			{
				// convert only if not already XYZ cloud
				bool hasField[4] = {false};
				for(size_t i=0; i<output->fields.size(); ++i)
				{
					if(output->fields[i].name.compare("x") == 0)
					{
						hasField[0] = true;
					}
					else if(output->fields[i].name.compare("y") == 0)
					{
						hasField[1] = true;
					}
					else if(output->fields[i].name.compare("z") == 0)
					{
						hasField[2] = true;
					}
					else
					{
						hasField[3] = true; // other
						break;
					}
				}
				if(hasField[0] && hasField[1] && hasField[2] && !hasField[3])
				{
					// do nothing, already XYZ
				}
				else
				{
					pcl::PointCloud<pcl::PointXYZ> cloudxyz;
					pcl::fromPCLPointCloud2(*output, cloudxyz);
					pcl::toPCLPointCloud2(cloudxyz, *output);
				}
			}

			for(unsigned int i=1; i<cloudMsgs.size(); ++i)
			{
				rtabmap::Transform cloudDisplacement;
				if(!fixedFrameId_.empty() &&
				   cloudMsgs[0]->header.stamp != cloudMsgs[i]->header.stamp)
				{
					// approx sync
					cloudDisplacement = rtabmap_conversions::getMovingTransform(
							frameId, //sourceTargetFrame
							fixedFrameId_, //fixedFrame
							cloudMsgs[0]->header.stamp, //stampTarget
							cloudMsgs[i]->header.stamp, //stampSource
							tfListener_,
							waitForTransformDuration_);
				}

				pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
				if(frameId.compare(cloudMsgs[i]->header.frame_id) != 0)
				{
					sensor_msgs::PointCloud2 tmp;
					pcl_ros::transformPointCloud(frameId, *cloudMsgs[i], tmp, tfListener_);
					if(!cloudDisplacement.isNull())
					{
						sensor_msgs::PointCloud2 tmp2;
						pcl_ros::transformPointCloud(cloudDisplacement.toEigen4f(), tmp, tmp2);
						pcl_conversions::toPCL(tmp2, *cloud2);
					}
					else
					{
						pcl_conversions::toPCL(tmp, *cloud2);
					}

				}
				else
				{
					if(!cloudDisplacement.isNull())
					{
						sensor_msgs::PointCloud2 tmp;
						pcl_ros::transformPointCloud(cloudDisplacement.toEigen4f(), *cloudMsgs[i], tmp);
						pcl_conversions::toPCL(tmp, *cloud2);
					}
					else
					{
						pcl_conversions::toPCL(*cloudMsgs[i], *cloud2);
					}
				}

				if(!cloud2->is_dense)
				{
					// remove nans
					cloud2 = rtabmap::util3d::removeNaNFromPointCloud(cloud2);
				}

				if(xyzOutput_ && !cloud2->data.empty())
				{
					// convert only if not already XYZ cloud
					bool hasField[4] = {false};
					for(size_t i=0; i<cloud2->fields.size(); ++i)
					{
						if(cloud2->fields[i].name.compare("x") == 0)
						{
							hasField[0] = true;
						}
						else if(cloud2->fields[i].name.compare("y") == 0)
						{
							hasField[1] = true;
						}
						else if(cloud2->fields[i].name.compare("z") == 0)
						{
							hasField[2] = true;
						}
						else
						{
							hasField[3] = true; // other
							break;
						}
					}
					if(hasField[0] && hasField[1] && hasField[2] && !hasField[3])
					{
						// do nothing, already XYZ
					}
					else
					{
						pcl::PointCloud<pcl::PointXYZ> cloudxyz;
						pcl::fromPCLPointCloud2(*cloud2, cloudxyz);
						pcl::toPCLPointCloud2(cloudxyz, *cloud2);
					}
				}

				if(output->data.empty())
				{
					output = cloud2;
				}
				else if(!cloud2->data.empty())
				{

					if(output->fields.size() != cloud2->fields.size())
					{
						ROS_WARN("%s: Input topics don't have all the "
								"same number of fields (cloud1=%d, cloud%d=%d), concatenation "
								"may fails. You can enable \"xyz_output\" option "
								"to convert all inputs to XYZ.",
								getName().c_str(),
								(int)output->fields.size(),
								i+1,
								(int)output->fields.size());
					}

					pcl::PCLPointCloud2::Ptr tmp_output(new pcl::PCLPointCloud2);
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
					pcl::concatenate(*output, *cloud2, *tmp_output);
#else
					pcl::concatenatePointCloud(*output, *cloud2, *tmp_output);
#endif
					//Make sure row_step is the sum of both
					tmp_output->row_step = tmp_output->width * tmp_output->point_step;
					output = tmp_output;
				}
			}

			sensor_msgs::PointCloud2 rosCloud;
			pcl_conversions::moveFromPCL(*output, rosCloud);
			rosCloud.header.stamp = cloudMsgs[0]->header.stamp;
			rosCloud.header.frame_id = frameId;
			cloudPub_.publish(rosCloud);
		}
	}

	void warningLoop(const std::string & subscribedTopicsMsg, bool approxSync)
	{
		ros::Duration r(5.0);
		while(!callbackCalled_)
		{
			r.sleep();
			if(!callbackCalled_)
			{
				ROS_WARN("%s: Did not receive data since 5 seconds! Make sure the input topics are "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set. %s%s",
						getName().c_str(),
						approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
							"topics should have all the exact timestamp for the callback to be called.",
						subscribedTopicsMsg.c_str());
			}
		}
	}

	boost::thread * warningThread_;
	bool callbackCalled_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactSync4Policy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxSync4Policy;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactSync3Policy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxSync3Policy;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactSync2Policy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxSync2Policy;
	message_filters::Synchronizer<ExactSync4Policy>* exactSync4_;
	message_filters::Synchronizer<ApproxSync4Policy>* approxSync4_;
	message_filters::Synchronizer<ExactSync3Policy>* exactSync3_;
	message_filters::Synchronizer<ApproxSync3Policy>* approxSync3_;
	message_filters::Synchronizer<ExactSync2Policy>* exactSync2_;
	message_filters::Synchronizer<ApproxSync2Policy>* approxSync2_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_1_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_2_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_3_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_4_;

	ros::Publisher cloudPub_;

	std::string frameId_;
	std::string fixedFrameId_;
	double waitForTransformDuration_;
	bool xyzOutput_;
	tf::TransformListener tfListener_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_util::PointCloudAggregator, nodelet::Nodelet);
}

