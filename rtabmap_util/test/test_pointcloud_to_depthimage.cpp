/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/pointcloud_to_depthimage.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

constexpr int kWidth = 16;
constexpr int kHeight = 16;
constexpr double kFx = 100.0;

float pixel32f(const sensor_msgs::msg::Image & img, int row, int col)
{
	return *reinterpret_cast<const float *>(&img.data[row * img.step + col * sizeof(float)]);
}

uint16_t pixel16u(const sensor_msgs::msg::Image & img, int row, int col)
{
	return *reinterpret_cast<const uint16_t *>(&img.data[row * img.step + col * sizeof(uint16_t)]);
}
}  // namespace

class PointCloudToDepthImageTest : public NodeTest
{
protected:
	void start(const std::vector<rclcpp::Parameter> & overrides = {}, bool withTf = true)
	{
		addNode(std::make_shared<rtabmap_util::PointCloudToDepthImage>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		if(withTf)
		{
			publishStaticTf("camera_link", "lidar");
		}
		image32_ = collect<sensor_msgs::msg::Image>("image");
		image16_ = collect<sensor_msgs::msg::Image>("image_raw");
		cloudPub_ = helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
		infoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(cloudPub_));
		ASSERT_TRUE(waitForSubscriber(infoPub_));
		ASSERT_TRUE(waitForPublisher(image32_->subscription));
	}

	/// Publishes a cloud and its camera info with identical stamps.
	void publishFrame(double stamp, const std::vector<cv::Point3f> & points)
	{
		cloudPub_->publish(makeXYZCloud("lidar", stamp, points));
		infoPub_->publish(makeCameraInfo("camera_link", stamp, kWidth, kHeight, 0.0, kFx));
	}

	/// A block of points straight ahead of the optical axis at @p depth meters.
	static std::vector<cv::Point3f> blockAt(float depth)
	{
		std::vector<cv::Point3f> points;
		for(int i=-2; i<=2; ++i)
		{
			for(int j=-2; j<=2; ++j)
			{
				points.push_back(cv::Point3f(0.01f*i, 0.01f*j, depth));
			}
		}
		return points;
	}

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> image32_;
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> image16_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;
};

TEST_F(PointCloudToDepthImageTest, ProjectsACloudIntoADepthImage)
{
	start();
	publishFrame(1000.0, blockAt(2.0f));
	ASSERT_TRUE(spinUntil([&]() { return !image32_->empty(); })) << "no depth image";

	const sensor_msgs::msg::Image & img = image32_->back();
	EXPECT_EQ(img.encoding, sensor_msgs::image_encodings::TYPE_32FC1);
	EXPECT_EQ(img.width, uint32_t(kWidth));
	EXPECT_EQ(img.height, uint32_t(kHeight));
	EXPECT_EQ(img.header.frame_id, "camera_link")
		<< "the depth image belongs to the camera, not the cloud";

	// The points sit on the optical axis, so they land on the principal point.
	EXPECT_NEAR(pixel32f(img, kHeight/2, kWidth/2), 2.0f, 1e-3);
	// A corner sees nothing.
	EXPECT_FLOAT_EQ(pixel32f(img, 0, 0), 0.0f);
}

TEST_F(PointCloudToDepthImageTest, PublishesMillimetersOnImageRaw)
{
	start();
	publishFrame(1000.0, blockAt(2.0f));
	ASSERT_TRUE(spinUntil([&]() { return !image16_->empty(); }));

	const sensor_msgs::msg::Image & img = image16_->back();
	EXPECT_EQ(img.encoding, sensor_msgs::image_encodings::TYPE_16UC1);
	EXPECT_EQ(pixel16u(img, kHeight/2, kWidth/2), 2000) << "2 m expressed in millimeters";
}

TEST_F(PointCloudToDepthImageTest, EmptyCloudGivesAnAllZeroImage)
{
	start();
	publishFrame(1000.0, {});
	ASSERT_TRUE(spinUntil([&]() { return !image32_->empty(); }))
		<< "an empty cloud must still produce an image, not a dropped frame";

	const sensor_msgs::msg::Image & img = image32_->back();
	EXPECT_EQ(img.width, uint32_t(kWidth));
	for(int row=0; row<kHeight; ++row)
	{
		for(int col=0; col<kWidth; ++col)
		{
			ASSERT_FLOAT_EQ(pixel32f(img, row, col), 0.0f) << "at " << row << "," << col;
		}
	}
}

TEST_F(PointCloudToDepthImageTest, DecimationShrinksTheImageAndScalesTheModel)
{
	start({rclcpp::Parameter("decimation", 2)});
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> infoOut =
			collect<sensor_msgs::msg::CameraInfo>("image/camera_info");
	ASSERT_TRUE(waitForPublisher(infoOut->subscription));

	publishFrame(1000.0, blockAt(2.0f));
	ASSERT_TRUE(spinUntil([&]() { return !image32_->empty() && !infoOut->empty(); }));

	EXPECT_EQ(image32_->back().width, uint32_t(kWidth)/2);
	EXPECT_EQ(image32_->back().height, uint32_t(kHeight)/2);
	EXPECT_NEAR(infoOut->back().p[0], kFx/2.0, 1e-6)
		<< "the published camera info must match the decimated image";
	EXPECT_EQ(infoOut->back().width, uint32_t(kWidth)/2);
}

TEST_F(PointCloudToDepthImageTest, FailsWithoutTheCloudToCameraTransform)
{
	start({rclcpp::Parameter("wait_for_transform", 0.0)}, /*withTf=*/false);
	publishFrame(1000.0, blockAt(2.0f));
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(image32_->empty())
		<< "without TF the cloud cannot be placed in the camera frame";
}

TEST_F(PointCloudToDepthImageTest, StaysSilentWithoutASubscriber)
{
	// The projection is skipped entirely when neither image topic is subscribed.
	addNode(std::make_shared<rtabmap_util::PointCloudToDepthImage>(rclcpp::NodeOptions()));
	publishStaticTf("camera_link", "lidar");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(cloudPub));

	cloudPub->publish(makeXYZCloud("lidar", 1000.0, blockAt(2.0f)));
	infoPub->publish(makeCameraInfo("camera_link", 1000.0, kWidth, kHeight, 0.0, kFx));
	spinFor(std::chrono::milliseconds(300));

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> late =
			collect<sensor_msgs::msg::Image>("image");
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(late->empty());
}

//============================================================================
// Motion compensation between the cloud stamp and the camera_info stamp
//============================================================================

/**
 * With approximate synchronization the cloud and the camera info rarely share a stamp.
 * When @c fixed_frame_id is set, the node asks TF how the lidar moved over that interval
 * and folds the displacement into the camera's local transform, so the cloud is projected
 * from where the camera was at its own stamp instead of where the lidar was.
 *
 * The frames follow the usual convention, as in rtabmap's own projectCloudToCamera tests:
 * the cloud is expressed in a lidar frame with x forward, and the camera is attached to it
 * through the optical rotation, so a point straight ahead lands on the principal point.
 */
class PointCloudToDepthImageMotionTest : public NodeTest
{
protected:
	static constexpr double kSpeed = 1.0;        ///< m/s
	static constexpr double kCloudStamp = 1000.0;
	static constexpr double kInfoDelay = 0.04;   ///< the camera info lags the cloud by this
	static constexpr float kRange = 2.0f;        ///< distance to the point, meters
	static constexpr int kFrames = 3;            ///< see publishFrames()
	static constexpr double kPeriod = 0.2;       ///< seconds between frames

	/// Distance travelled between the two stamps: what the node has to compensate for.
	static double travelled() { return kSpeed * kInfoDelay; }

	/// The same distance seen sideways by the camera, in pixels.
	static int shiftInPixels() { return int(kFx * travelled() / double(kRange)); }

	/**
	 * @param axis  'x' to drive straight at the point, 'y' to drive sideways past it,
	 *              '0' to stand still
	 */
	void start(const std::vector<rclcpp::Parameter> & overrides, char axis = 'x')
	{
		addNode(std::make_shared<rtabmap_util::PointCloudToDepthImage>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		// The camera is bolted to the lidar, looking the same way: x right, y down,
		// z forward against the lidar's x forward, y left, z up.
		publishStaticTfRPY("lidar", "camera_link", -M_PI/2.0, 0.0, -M_PI/2.0);
		if(axis != '0')
		{
			publishOdomMotion(axis);
		}
		image32_ = collect<sensor_msgs::msg::Image>("image");
		cloudPub_ = helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
		infoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(cloudPub_));
		ASSERT_TRUE(waitForSubscriber(infoPub_));
		ASSERT_TRUE(waitForPublisher(image32_->subscription));
	}

	/// Publishes odom -> lidar moving at kSpeed, covering every stamp used below.
	void publishOdomMotion(char axis)
	{
		rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPub =
				helper()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
		spinFor(std::chrono::milliseconds(100));   // let the node's listener subscribe

		// One sample per cloud stamp and per camera info stamp, plus a margin on each side
		// so that nothing has to be extrapolated.
		std::vector<double> elapsedSamples;
		elapsedSamples.push_back(-kPeriod);
		for(int k=0; k<kFrames; ++k)
		{
			elapsedSamples.push_back(kPeriod * double(k));
			elapsedSamples.push_back(kPeriod * double(k) + kInfoDelay);
		}
		elapsedSamples.push_back(kPeriod * double(kFrames));

		for(size_t i=0; i<elapsedSamples.size(); ++i)
		{
			const double elapsed = elapsedSamples[i];
			geometry_msgs::msg::TransformStamped t;
			t.header.stamp = stampOf(kCloudStamp + elapsed);
			t.header.frame_id = "odom";
			t.child_frame_id = "lidar";
			(axis == 'x' ? t.transform.translation.x : t.transform.translation.y) =
					kSpeed * elapsed;
			t.transform.rotation.w = 1.0;
			tf2_msgs::msg::TFMessage msg;
			msg.transforms.push_back(t);
			tfPub->publish(msg);
		}
		spinFor(std::chrono::milliseconds(200));   // let the buffer fill
		tfPub_ = tfPub;                            // keep the publisher alive
	}

	/**
	 * @brief Publishes kFrames cloud/camera_info pairs, the info @p delay seconds late.
	 *
	 * Each cloud holds a single point straight ahead of the lidar. The speed is constant,
	 * so every pair needs the same correction and the first output is enough to assert on.
	 * A burst is needed because ApproximateTime emits nothing for a lone pair whose stamps
	 * differ: it cannot rule out a better match still to come.
	 */
	void publishFrames(double delay)
	{
		for(int k=0; k<kFrames; ++k)
		{
			const double elapsed = kPeriod * double(k);
			cloudPub_->publish(makeXYZCloud("lidar", kCloudStamp + elapsed,
					{cv::Point3f(kRange, 0.0f, 0.0f)}));
			infoPub_->publish(makeCameraInfo(
					"camera_link", kCloudStamp + elapsed + delay, kWidth, kHeight, 0.0, kFx));
		}
	}

	/// The column the single point landed in on @p row, or -1 if that row is empty.
	static int hitColumn(const sensor_msgs::msg::Image & img, int row)
	{
		for(int col=0; col<int(img.width); ++col)
		{
			if(pixel32f(img, row, col) != 0.0f) { return col; }
		}
		return -1;
	}

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> image32_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPub_;
};

TEST_F(PointCloudToDepthImageMotionTest, NoShiftWhenTheStampsMatch)
{
	// The control case: same frames, same scene, nothing to compensate. It also proves the
	// optical rotation is right, since a wrongly oriented camera sees nothing at all.
	start({rclcpp::Parameter("fixed_frame_id", std::string("odom"))});
	publishFrames(0.0);
	ASSERT_TRUE(spinUntil([&]() { return !image32_->empty(); })) << "no depth image";

	const sensor_msgs::msg::Image & img = image32_->front();
	EXPECT_EQ(hitColumn(img, kHeight/2), kWidth/2)
		<< "a point straight ahead belongs at the principal point";
	EXPECT_NEAR(pixel32f(img, kHeight/2, kWidth/2), kRange, 1e-3);
}

TEST_F(PointCloudToDepthImageMotionTest, ClosesTheGapWhenDrivingAtThePoint)
{
	// The camera info is 40 ms younger than the cloud and the robot closes in at 1 m/s, so
	// by the time of the exposure the point is 4 cm nearer than the lidar measured it.
	start({rclcpp::Parameter("fixed_frame_id", std::string("odom"))}, 'x');
	publishFrames(kInfoDelay);
	ASSERT_TRUE(spinUntil([&]() { return !image32_->empty(); })) << "no depth image";

	const sensor_msgs::msg::Image & img = image32_->front();
	EXPECT_EQ(hitColumn(img, kHeight/2), kWidth/2)
		<< "driving straight at the point does not move it across the image";
	EXPECT_NEAR(pixel32f(img, kHeight/2, kWidth/2), kRange - float(travelled()), 1e-3)
		<< "the depth must be corrected for the distance travelled";
}

TEST_F(PointCloudToDepthImageMotionTest, ShiftsThePointWhenDrivingPastIt)
{
	// Moving sideways instead: the point slides across the image by fx*d/Z pixels, and
	// stays at the same range.
	start({rclcpp::Parameter("fixed_frame_id", std::string("odom"))}, 'y');
	publishFrames(kInfoDelay);
	ASSERT_TRUE(spinUntil([&]() { return !image32_->empty(); })) << "no depth image";

	const sensor_msgs::msg::Image & img = image32_->front();
	EXPECT_EQ(hitColumn(img, kHeight/2), kWidth/2 + shiftInPixels())
		<< "the robot moved left, so the point must appear further right";
	EXPECT_NEAR(pixel32f(img, kHeight/2, kWidth/2 + shiftInPixels()), kRange, 1e-3)
		<< "only the bearing changed, not the range";
	EXPECT_FLOAT_EQ(pixel32f(img, kHeight/2, kWidth/2), 0.0f)
		<< "and it is no longer at the principal point";
}

TEST_F(PointCloudToDepthImageMotionTest, IgnoresTheStampDifferenceWithoutAFixedFrame)
{
	// Without fixed_frame_id there is nothing to measure the motion against, so the cloud
	// is projected as if both messages were captured at the same instant. That is why the
	// node logs a fatal error when approximate sync is used without one.
	start({rclcpp::Parameter("fixed_frame_id", std::string(""))}, 'x');
	publishFrames(kInfoDelay);
	ASSERT_TRUE(spinUntil([&]() { return !image32_->empty(); })) << "no depth image";

	EXPECT_NEAR(pixel32f(image32_->front(), kHeight/2, kWidth/2), kRange, 1e-3)
		<< "no fixed frame, no compensation";
}

TEST_F(PointCloudToDepthImageMotionTest, FailsWhenTheFixedFrameIsUnknown)
{
	// fixed_frame_id is set but odom -> lidar was never published: the displacement cannot
	// be measured, and projecting anyway would silently misplace the points.
	start({rclcpp::Parameter("fixed_frame_id", std::string("odom")),
		   rclcpp::Parameter("wait_for_transform", 0.0)}, '0');
	publishFrames(kInfoDelay);
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(image32_->empty());
}
