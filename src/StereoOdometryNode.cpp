/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "OdometryROS.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/video/tracking.hpp>

#include "rtabmap/MsgConversion.h"

#include <rtabmap/core/Features2d.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMath.h>

using namespace rtabmap;

class StereoOdometry : public OdometryROS
{
public:
	StereoOdometry(int argc, char * argv[]) :
		OdometryROS(argc, argv),
		feature2D_(0),
		depthPatchSize_(1),
		generateDepth_(false),
		stereoFlowWinSize_(21),
		stereoFlowIterations_(30),
		stereoFlowEpsilon_(0.01),
		stereoFlowMaxLevel_(3),
		stereoSubPixWinSize_(5),
		stereoSubPixIterations_(20),
		stereoSubPixEps_(0.03),
		approxSync_(0),
		exactSync_(0)
	{
		ros::NodeHandle nh;

		odomDepth_ = nh.advertise<sensor_msgs::Image>("odom_depth", 1);

		ros::NodeHandle pnh("~");

		bool approxSync = false;
		int queueSize = 5;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("queue_size", queueSize, queueSize);
		ROS_INFO("Approximate time sync = %s", approxSync?"true":"false");

		pnh.param("generate_depth", generateDepth_, generateDepth_);
		pnh.param("depth_patch_size", depthPatchSize_, depthPatchSize_);
		ROS_INFO("Generate depth = %s", generateDepth_?"true":"false");

		pnh.param("flow_win_size", stereoFlowWinSize_, stereoFlowWinSize_);
		pnh.param("flow_iterations", stereoFlowIterations_, stereoFlowIterations_);
		pnh.param("flow_epsilon", stereoFlowEpsilon_, stereoFlowEpsilon_);
		pnh.param("flow_max_level", stereoFlowMaxLevel_, stereoFlowMaxLevel_);

		pnh.param("subpix_win_size", stereoSubPixWinSize_, stereoSubPixWinSize_);
		pnh.param("subpix_iterations", stereoSubPixIterations_, stereoSubPixIterations_);
		pnh.param("subpix_eps", stereoSubPixEps_, stereoSubPixEps_);

		UASSERT_MSG(!this->isOdometryBOW() || (this->isOdometryBOW() && generateDepth_),
				"Odom/Strategy=0 (OdometryBOW) requires depth generation (generate_depth=true).");

		UASSERT(depthPatchSize_ >= 0);

		//Keypoint detector
		ParametersMap::const_iterator iter;
		Feature2D::Type detectorStrategy = Feature2D::kFeatureUndef;
		if((iter=this->parameters().find(Parameters::kOdomFeatureType())) != this->parameters().end())
		{
			detectorStrategy = (Feature2D::Type)std::atoi((*iter).second.c_str());
		}
		feature2D_ = Feature2D::create(detectorStrategy, this->parameters());

		roiRatios_ = Parameters::defaultOdomRoiRatios();
		Parameters::parse(this->parameters(), Parameters::kOdomRoiRatios(), roiRatios_);

		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport left_it(left_nh);
		image_transport::ImageTransport right_it(right_nh);
		image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

		imageRectLeft_.subscribe(left_it, left_nh.resolveName("image_rect"), 1, hintsLeft);
		imageRectRight_.subscribe(right_it, right_nh.resolveName("image_rect"), 1, hintsRight);
		cameraInfoLeft_.subscribe(left_nh, "camera_info", 1);
		cameraInfoRight_.subscribe(right_nh, "camera_info", 1);

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			approxSync_->registerCallback(boost::bind(&StereoOdometry::callback, this, _1, _2, _3, _4));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			exactSync_->registerCallback(boost::bind(&StereoOdometry::callback, this, _1, _2, _3, _4));
		}
	}

	~StereoOdometry()
	{
		if(approxSync_)
		{
			delete approxSync_;
		}
		if(exactSync_)
		{
			delete exactSync_;
		}
		delete feature2D_;
	}

	void callback(
			const sensor_msgs::ImageConstPtr& imageRectLeft,
			const sensor_msgs::ImageConstPtr& imageRectRight,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoLeft,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoRight)
	{
		if(!this->isPaused())
		{
			if(!(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
				!(imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
			{
				ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 recommended)");
				return;
			}

			tf::StampedTransform localTransform;
			try
			{
				UDEBUG("");
				this->tfListener().lookupTransform(this->frameId(), imageRectLeft->header.frame_id, imageRectLeft->header.stamp, localTransform);
			}
			catch(tf::TransformException & ex)
			{
				ROS_WARN("%s",ex.what());
				return;
			}

			ros::WallTime time = ros::WallTime::now();

			int quality = -1;
			if(imageRectLeft->data.size() && imageRectRight->data.size())
			{
				image_geometry::StereoCameraModel model;
				model.fromCameraInfo(*cameraInfoLeft, *cameraInfoRight);

				float fx = model.left().fx();
				float fy = model.left().fy();
				float cx = model.left().cx();
				float cy = model.left().cy();
				float baseline = model.baseline();
				cv_bridge::CvImageConstPtr ptrImageLeft = cv_bridge::toCvShare(imageRectLeft, "mono8");
				cv_bridge::CvImageConstPtr ptrImageRight = cv_bridge::toCvShare(imageRectRight, "mono8");

				cv::Mat depthOrRightImage;
				std::vector<cv::KeyPoint> kptsLeft, kptsRight;
				cv::Mat descLeft, descRight;
				UTimer stepTimer;

				if(!generateDepth_)
				{
					// copy right image in depth
					depthOrRightImage = ptrImageRight->image;
				}
				else
				{
					//generate depth
					depthOrRightImage = cv::Mat::zeros(ptrImageLeft->image.rows, ptrImageLeft->image.cols, CV_32FC1);

					std::vector<cv::Point2f> cornersLeft, cornersRight;

					cv::Rect roi = Feature2D::computeRoi(ptrImageLeft->image, roiRatios_);
					kptsLeft = feature2D_->generateKeypoints(ptrImageLeft->image, 0, roi);
					UDEBUG("time generate left kpts=%fs", stepTimer.ticks());

					if(!kptsLeft.size())
					{
						ROS_WARN("No left keypoints extracted!");
						return;
					}

					int stereoFeaturesAdded = 0;
					int stereoFeaturesMatched = 0;
					int stereoFeaturesExtracted = 0;

					cv::KeyPoint::convert(kptsLeft, cornersLeft);

					if(stereoSubPixWinSize_ > 0 && stereoSubPixIterations_ > 0)
					{
						cv::cornerSubPix( ptrImageLeft->image, cornersLeft,
								cv::Size( stereoSubPixWinSize_, stereoSubPixWinSize_ ),
								cv::Size( -1, -1 ),
								cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, stereoSubPixIterations_, stereoSubPixEps_ ) );
						UDEBUG("time subpix left kpts=%fs", stepTimer.ticks());
					}

					std::vector<unsigned char> status;
					std::vector<float> err;

					UDEBUG("cv::calcOpticalFlowPyrLK() begin");
					cv::calcOpticalFlowPyrLK(
							ptrImageLeft->image,
							ptrImageRight->image,
							cornersLeft,
							cornersRight,
							status,
							err,
							cv::Size(stereoFlowWinSize_, stereoFlowWinSize_), stereoFlowMaxLevel_,
							cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, stereoFlowIterations_, stereoFlowEpsilon_),
							cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
					UDEBUG("cv::calcOpticalFlowPyrLK() end");
					UDEBUG("time optical flow=%fs", stepTimer.ticks());

					std::vector<cv::KeyPoint> kptsLeftFiltered(kptsLeft.size());
					int oi = 0;
					for(int i=0; i<status.size(); ++i)
					{
						if(status[i] &&
							uIsInBounds(cornersLeft[i].x, 0.0f, float(depthOrRightImage.cols)-1.0f) &&
							uIsInBounds(cornersLeft[i].y, 0.0f, float(depthOrRightImage.rows)-1.0f) &&
							uIsInBounds(cornersRight[i].x, 0.0f, float(depthOrRightImage.cols)-1.0f) &&
							uIsInBounds(cornersRight[i].y, 0.0f, float(depthOrRightImage.rows)-1.0f))
						{
							float disparity = cornersLeft[i].x - cornersRight[i].x;

							if(disparity >= 0)
							{
								float d = model.getZ(disparity);
								if(d>0)
								{
									bool depthAdded = false;
									int u = int(cornersLeft[i].x+0.5f);
									int v = int(cornersLeft[i].y+0.5f);
									for(int j=-depthPatchSize_; j<=depthPatchSize_; ++j)
									{
										for(int k=-depthPatchSize_; k<=depthPatchSize_; ++k)
										{
											if(uIsInBounds(u+j, 0, depthOrRightImage.cols-1) &&
											   uIsInBounds(v+k, 0, depthOrRightImage.rows-1))
											{
												depthOrRightImage.at<float>(v+j, u+k) = d;
												depthAdded = true;
											}
										}
									}
									if(depthAdded)
									{
										kptsLeftFiltered[oi] = kptsLeft[i];
										kptsLeftFiltered[oi].pt = cornersLeft[i];
										++oi;
									}
								}
							}
							++stereoFeaturesMatched;
						}
					}
					stereoFeaturesAdded = oi;
					stereoFeaturesExtracted = kptsLeft.size();

					UDEBUG("stereoFeaturesExtracted=%d", stereoFeaturesExtracted);
					UDEBUG("stereoFeaturesMatched=%d", stereoFeaturesMatched);
					UDEBUG("stereoFeaturesAdded=%d", stereoFeaturesAdded);

					kptsLeftFiltered.resize(oi);
					kptsLeft = kptsLeftFiltered;

					if(!kptsLeft.size())
					{
						ROS_WARN("No left keypoints extracted!");
						return;
					}

					// For OdometryBOW, we must generate descriptors
					int odomStrategy = Parameters::defaultOdomStrategy();
					Parameters::parse(this->parameters(), Parameters::kOdomStrategy(), odomStrategy);
					if(odomStrategy == 0)
					{
						descLeft = feature2D_->generateDescriptors(ptrImageLeft->image, kptsLeft);
						UDEBUG("time generate left descriptors=%fs, remaining kpts=%d", stepTimer.ticks(), (int)kptsLeft.size());
						if(!kptsLeft.size())
						{
							ROS_WARN("No left descriptors extracted!");
							return;
						}
					}
				}

				//
				UDEBUG("localTransform = %s", rtabmap::transformFromTF(localTransform).prettyPrint().c_str());
				UDEBUG("kptsLeft=%d descLeft=%d", (int)kptsLeft.size(), descLeft.rows);
				rtabmap::SensorData data(ptrImageLeft->image,
						depthOrRightImage,
						fx,
						generateDepth_?fy:baseline,
						cx,
						cy,
						rtabmap::Transform(),
						rtabmap::transformFromTF(localTransform));
				data.setFeatures(kptsLeft, descLeft);
				quality=0;

				this->processData(data, imageRectLeft->header, quality);
				UDEBUG("time odometry->process()=%fs", stepTimer.ticks());

				if(generateDepth_ && odomDepth_.getNumSubscribers())
				{
					cv_bridge::CvImage img;
					img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
					img.image = depthOrRightImage;
					sensor_msgs::ImagePtr rosMsg = img.toImageMsg();
					rosMsg->header= imageRectLeft->header;
					odomDepth_.publish(rosMsg);
				}

				//ROS_INFO("Odom: quality=%d, update time=%fs, stereo matches: added/matched/extracted %d/%d/%d",
				//	quality, (ros::WallTime::now()-time).toSec(),
				//	stereoFeaturesAdded, stereoFeaturesMatched, stereoFeaturesExtracted);
				ROS_INFO("Odom: quality=%d, update time=%fs",
						quality, (ros::WallTime::now()-time).toSec());
			}
			else
			{
				ROS_WARN("Odom: input images empty?!?");
			}
		}
	}

private:
	Feature2D * feature2D_;
	std::string roiRatios_;

	// ROS parameters
	int depthPatchSize_;

	bool generateDepth_;

	int stereoFlowWinSize_;
	int stereoFlowIterations_;
	double stereoFlowEpsilon_;
	int stereoFlowMaxLevel_;

	int stereoSubPixWinSize_;
	int stereoSubPixIterations_;
	double stereoSubPixEps_;

	ros::Publisher odomDepth_;

	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

int main(int argc, char *argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);
	//pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
	ros::init(argc, argv, "stereo_odometry");

	StereoOdometry odom(argc, argv);
	ros::spin();
	return 0;
}
