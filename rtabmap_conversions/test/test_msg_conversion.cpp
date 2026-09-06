/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>

#include <rtabmap_conversions/MsgConversion.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/Compression.h>

using namespace rtabmap_conversions;

namespace {

// A transform with translation and rotation on all three axes, so that a
// round-trip that drops or swaps a component cannot pass by accident.
rtabmap::Transform sampleTransform()
{
	return rtabmap::Transform(1.0f, -2.0f, 3.0f, 0.1f, -0.2f, 0.3f);
}

void expectTransformNear(
		const rtabmap::Transform & actual,
		const rtabmap::Transform & expected,
		float epsilon = 1e-5f)
{
	ASSERT_FALSE(actual.isNull()) << "expected " << expected.prettyPrint();
	for(int i=0; i<12; ++i)
	{
		EXPECT_NEAR(actual.data()[i], expected.data()[i], epsilon)
			<< "at index " << i
			<< "\n  actual:   " << actual.prettyPrint()
			<< "\n  expected: " << expected.prettyPrint();
	}
}

}  // namespace

/////////////////////////
// Transform <-> geometry_msgs
/////////////////////////

TEST(MsgConversion, transformGeometryMsgRoundTrip)
{
	const rtabmap::Transform in = sampleTransform();

	geometry_msgs::msg::Transform msg;
	transformToGeometryMsg(in, msg);

	expectTransformNear(transformFromGeometryMsg(msg), in);
}

TEST(MsgConversion, transformGeometryMsgQuaternionIsNormalized)
{
	geometry_msgs::msg::Transform msg;
	transformToGeometryMsg(sampleTransform(), msg);

	const double norm = std::sqrt(
			msg.rotation.x * msg.rotation.x +
			msg.rotation.y * msg.rotation.y +
			msg.rotation.z * msg.rotation.z +
			msg.rotation.w * msg.rotation.w);
	EXPECT_NEAR(norm, 1.0, 1e-9);
}

TEST(MsgConversion, transformGeometryMsgNullRoundTrip)
{
	geometry_msgs::msg::Transform msg;
	transformToGeometryMsg(rtabmap::Transform(), msg);

	// A null transform is encoded as an all-zero quaternion.
	EXPECT_EQ(msg.rotation.x, 0.0);
	EXPECT_EQ(msg.rotation.y, 0.0);
	EXPECT_EQ(msg.rotation.z, 0.0);
	EXPECT_EQ(msg.rotation.w, 0.0);
	EXPECT_TRUE(transformFromGeometryMsg(msg).isNull());
}

TEST(MsgConversion, transformGeometryMsgIdentityIsNotNull)
{
	geometry_msgs::msg::Transform msg;
	transformToGeometryMsg(rtabmap::Transform::getIdentity(), msg);

	const rtabmap::Transform out = transformFromGeometryMsg(msg);
	EXPECT_FALSE(out.isNull());
	EXPECT_TRUE(out.isIdentity());
}

/////////////////////////
// Transform <-> tf2
/////////////////////////

TEST(MsgConversion, transformTFRoundTrip)
{
	const rtabmap::Transform in = sampleTransform();

	tf2::Transform tf;
	EXPECT_TRUE(transformToTF(in, tf));

	expectTransformNear(transformFromTF(tf), in);
}

TEST(MsgConversion, transformTFIdentityRoundTrip)
{
	tf2::Transform tf;
	EXPECT_TRUE(transformToTF(rtabmap::Transform::getIdentity(), tf))
		<< "an identity transform is not a null transform";

	const rtabmap::Transform out = transformFromTF(tf);
	EXPECT_FALSE(out.isNull());
	EXPECT_TRUE(out.isIdentity());
}

TEST(MsgConversion, transformToTFWritesTranslationAndRotation)
{
	// Guards against the output being left untouched: seed it with a value that
	// differs from the expected result, then check it was actually overwritten.
	tf2::Transform tf(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(99, 99, 99));
	EXPECT_TRUE(transformToTF(sampleTransform(), tf));

	EXPECT_NEAR(tf.getOrigin().x(), 1.0, 1e-5);
	EXPECT_NEAR(tf.getOrigin().y(), -2.0, 1e-5);
	EXPECT_NEAR(tf.getOrigin().z(), 3.0, 1e-5);

	geometry_msgs::msg::Transform expected;
	transformToGeometryMsg(sampleTransform(), expected);
	EXPECT_NEAR(tf.getRotation().x(), expected.rotation.x, 1e-5);
	EXPECT_NEAR(tf.getRotation().y(), expected.rotation.y, 1e-5);
	EXPECT_NEAR(tf.getRotation().z(), expected.rotation.z, 1e-5);
	EXPECT_NEAR(tf.getRotation().w(), expected.rotation.w, 1e-5);
}

TEST(MsgConversion, transformToTFNullReturnsFalseAndNaN)
{
	tf2::Transform tf(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(99, 99, 99));

	EXPECT_FALSE(transformToTF(rtabmap::Transform(), tf));

	// tf2::Transform cannot carry a null sentinel, so the output is poisoned with NaN
	// on purpose: a caller that ignores the return value must fail loudly rather than
	// silently proceed with a plausible-looking identity.
	for(int i=0; i<3; ++i)
	{
		EXPECT_TRUE(std::isnan(tf.getBasis()[i].x())) << "basis row " << i;
		EXPECT_TRUE(std::isnan(tf.getBasis()[i].y())) << "basis row " << i;
		EXPECT_TRUE(std::isnan(tf.getBasis()[i].z())) << "basis row " << i;
	}
	EXPECT_TRUE(std::isnan(tf.getOrigin().x()));
	EXPECT_TRUE(std::isnan(tf.getOrigin().y()));
	EXPECT_TRUE(std::isnan(tf.getOrigin().z()));

	const tf2::Quaternion q = tf.getRotation();
	EXPECT_TRUE(std::isnan(q.x()));
	EXPECT_TRUE(std::isnan(q.y()));
	EXPECT_TRUE(std::isnan(q.z()));
	EXPECT_TRUE(std::isnan(q.w()));
}

TEST(MsgConversion, transformToTFNullPoisonsComposition)
{
	// The point of the NaN: it propagates through downstream math instead of
	// quietly producing a wrong-but-finite answer.
	tf2::Transform tf;
	EXPECT_FALSE(transformToTF(rtabmap::Transform(), tf));

	const tf2::Transform composed =
			tf * tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(1, 2, 3));

	EXPECT_TRUE(std::isnan(composed.getOrigin().x()));
	EXPECT_TRUE(std::isnan(composed.getOrigin().y()));
	EXPECT_TRUE(std::isnan(composed.getOrigin().z()));
}

TEST(MsgConversion, transformFromTFDetectsNaN)
{
	const tf2Scalar nan = std::numeric_limits<tf2Scalar>::quiet_NaN();

	// NaN anywhere in the rotation basis...
	EXPECT_TRUE(transformFromTF(tf2::Transform(
			tf2::Matrix3x3(nan, nan, nan, nan, nan, nan, nan, nan, nan),
			tf2::Vector3(0, 0, 0))).isNull()) << "NaN basis";

	// ...or in the translation alone must yield a null transform.
	EXPECT_TRUE(transformFromTF(tf2::Transform(
			tf2::Quaternion(0, 0, 0, 1),
			tf2::Vector3(nan, 0, 0))).isNull()) << "NaN origin";
}

TEST(MsgConversion, transformTFNullRoundTrip)
{
	// The pair round-trips a null transform: toTF poisons with NaN and reports
	// false, fromTF maps that back to null.
	tf2::Transform tf;
	EXPECT_FALSE(transformToTF(rtabmap::Transform(), tf));
	EXPECT_TRUE(transformFromTF(tf).isNull());
}

TEST(MsgConversion, transformFromTFAcceptsValidTransforms)
{
	// The NaN guard must not reject legitimate values, including zeros.
	EXPECT_FALSE(transformFromTF(tf2::Transform(
			tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0))).isNull());
	EXPECT_FALSE(transformFromTF(tf2::Transform(
			tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(-1, 2, -3))).isNull());
}

/////////////////////////
// Transform <-> Pose
/////////////////////////

TEST(MsgConversion, transformPoseMsgRoundTrip)
{
	const rtabmap::Transform in = sampleTransform();

	geometry_msgs::msg::Pose msg;
	transformToPoseMsg(in, msg);

	expectTransformNear(transformFromPoseMsg(msg), in);
}

TEST(MsgConversion, transformPoseMsgNullIsNull)
{
	geometry_msgs::msg::Pose msg;
	transformToPoseMsg(rtabmap::Transform(), msg);

	EXPECT_TRUE(transformFromPoseMsg(msg).isNull());
}

TEST(MsgConversion, transformPoseMsgIgnoreRotationIfNotSet)
{
	// Note: geometry_msgs::msg::Quaternion defaults to w=1.0, so an "unset"
	// orientation has to be zeroed explicitly to reach the branch under test.
	geometry_msgs::msg::Pose msg;
	msg.position.x = 1.0;
	msg.position.y = 2.0;
	msg.position.z = 3.0;
	msg.orientation.w = 0.0;

	// An all-zero orientation normally yields a null transform...

	EXPECT_TRUE(transformFromPoseMsg(msg, false).isNull());

	// ...but with ignoreRotationIfNotSet the translation is kept with no rotation.
	expectTransformNear(
			transformFromPoseMsg(msg, true),
			rtabmap::Transform(1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f));
}

/////////////////////////
// Points and keypoints
/////////////////////////

TEST(MsgConversion, point2fRoundTrip)
{
	const cv::Point2f in(1.5f, -2.5f);

	rtabmap_msgs::msg::Point2f msg;
	point2fToROS(in, msg);
	const cv::Point2f out = point2fFromROS(msg);

	EXPECT_FLOAT_EQ(out.x, in.x);
	EXPECT_FLOAT_EQ(out.y, in.y);
}

TEST(MsgConversion, points2fVectorRoundTrip)
{
	const std::vector<cv::Point2f> in = {{1.0f, 2.0f}, {-3.0f, 4.5f}};

	std::vector<rtabmap_msgs::msg::Point2f> msg;
	points2fToROS(in, msg);
	const std::vector<cv::Point2f> out = points2fFromROS(msg);

	ASSERT_EQ(out.size(), in.size());
	for(size_t i=0; i<in.size(); ++i)
	{
		EXPECT_FLOAT_EQ(out[i].x, in[i].x) << "at " << i;
		EXPECT_FLOAT_EQ(out[i].y, in[i].y) << "at " << i;
	}
}

TEST(MsgConversion, point3fRoundTrip)
{
	const cv::Point3f in(1.5f, -2.5f, 3.5f);

	rtabmap_msgs::msg::Point3f msg;
	point3fToROS(in, msg);
	const cv::Point3f out = point3fFromROS(msg);

	EXPECT_FLOAT_EQ(out.x, in.x);
	EXPECT_FLOAT_EQ(out.y, in.y);
	EXPECT_FLOAT_EQ(out.z, in.z);
}

TEST(MsgConversion, points3fVectorRoundTripWithoutTransform)
{
	const std::vector<cv::Point3f> in = {{1.0f, 2.0f, 3.0f}, {-4.0f, 5.0f, -6.0f}};

	std::vector<rtabmap_msgs::msg::Point3f> msg;
	points3fToROS(in, msg);
	const std::vector<cv::Point3f> out = points3fFromROS(msg);

	ASSERT_EQ(out.size(), in.size());
	for(size_t i=0; i<in.size(); ++i)
	{
		EXPECT_FLOAT_EQ(out[i].x, in[i].x) << "at " << i;
		EXPECT_FLOAT_EQ(out[i].y, in[i].y) << "at " << i;
		EXPECT_FLOAT_EQ(out[i].z, in[i].z) << "at " << i;
	}
}

TEST(MsgConversion, points3fTransformIsAppliedOnBothDirections)
{
	const std::vector<cv::Point3f> in = {{1.0f, 2.0f, 3.0f}};
	const rtabmap::Transform t = sampleTransform();

	// Applying t on the way out and t.inverse() on the way in must cancel.
	std::vector<rtabmap_msgs::msg::Point3f> msg;
	points3fToROS(in, msg, t);
	const std::vector<cv::Point3f> out = points3fFromROS(msg, t.inverse());

	ASSERT_EQ(out.size(), in.size());
	EXPECT_NEAR(out[0].x, in[0].x, 1e-4);
	EXPECT_NEAR(out[0].y, in[0].y, 1e-4);
	EXPECT_NEAR(out[0].z, in[0].z, 1e-4);

	// ...and the intermediate message really is the transformed point.
	const cv::Point3f expected = rtabmap::util3d::transformPoint(in[0], t);
	EXPECT_NEAR(msg[0].x, expected.x, 1e-4);
	EXPECT_NEAR(msg[0].y, expected.y, 1e-4);
	EXPECT_NEAR(msg[0].z, expected.z, 1e-4);
}

TEST(MsgConversion, points3fFromROSAppendsToExistingVector)
{
	std::vector<rtabmap_msgs::msg::Point3f> msg(2);
	msg[0].x = 1.0f;
	msg[1].x = 2.0f;

	std::vector<cv::Point3f> points = {{9.0f, 9.0f, 9.0f}};
	points3fFromROS(msg, points);

	ASSERT_EQ(points.size(), 3u);
	EXPECT_FLOAT_EQ(points[0].x, 9.0f) << "existing content must be preserved";
	EXPECT_FLOAT_EQ(points[1].x, 1.0f);
	EXPECT_FLOAT_EQ(points[2].x, 2.0f);
}

TEST(MsgConversion, keypointRoundTrip)
{
	const cv::KeyPoint in(cv::Point2f(10.0f, 20.0f), 7.0f, 45.0f, 0.5f, 2, 3);

	rtabmap_msgs::msg::KeyPoint msg;
	keypointToROS(in, msg);
	const cv::KeyPoint out = keypointFromROS(msg);

	EXPECT_FLOAT_EQ(out.pt.x, in.pt.x);
	EXPECT_FLOAT_EQ(out.pt.y, in.pt.y);
	EXPECT_FLOAT_EQ(out.size, in.size);
	EXPECT_FLOAT_EQ(out.angle, in.angle);
	EXPECT_FLOAT_EQ(out.response, in.response);
	EXPECT_EQ(out.octave, in.octave);
	EXPECT_EQ(out.class_id, in.class_id);
}

TEST(MsgConversion, keypointsFromROSAppendsAndAppliesXShift)
{
	const std::vector<cv::KeyPoint> in = {
		cv::KeyPoint(cv::Point2f(10.0f, 20.0f), 7.0f),
		cv::KeyPoint(cv::Point2f(30.0f, 40.0f), 7.0f)};

	std::vector<rtabmap_msgs::msg::KeyPoint> msg;
	keypointsToROS(in, msg);
	ASSERT_EQ(msg.size(), in.size());

	std::vector<cv::KeyPoint> kpts = {cv::KeyPoint(cv::Point2f(1.0f, 1.0f), 1.0f)};
	keypointsFromROS(msg, kpts, /*xShift=*/100);

	ASSERT_EQ(kpts.size(), 3u);
	EXPECT_FLOAT_EQ(kpts[0].pt.x, 1.0f) << "existing content must be preserved";
	EXPECT_FLOAT_EQ(kpts[1].pt.x, 110.0f);
	EXPECT_FLOAT_EQ(kpts[2].pt.x, 130.0f);
	EXPECT_FLOAT_EQ(kpts[1].pt.y, 20.0f) << "xShift must not touch y";
}

/////////////////////////
// Timestamps
/////////////////////////

TEST(MsgConversion, timestampRoundTrip)
{
	const double in = 1234567890.123456;
	EXPECT_NEAR(timestampFromROS(timestampToROS(in)), in, 1e-6);
}

TEST(MsgConversion, timestampToROSUsesRosClock)
{
	// Message header stamps convert to RCL_ROS_TIME, while rclcpp::Time(sec, nsec)
	// defaults to RCL_SYSTEM_TIME. Comparing two different clock types throws, so a
	// timestamp built here must be comparable with one taken from a message -- several
	// conversions do exactly that when syncing to an odometry stamp.
	const rclcpp::Time built = timestampToROS(1000.0);
	EXPECT_EQ(built.get_clock_type(), RCL_ROS_TIME);

	builtin_interfaces::msg::Time asMsg = timestampToROS(1000.5);
	const rclcpp::Time fromMsg(asMsg);
	EXPECT_EQ(fromMsg.get_clock_type(), RCL_ROS_TIME);

	EXPECT_NO_THROW({ volatile bool differ = (built != fromMsg); (void)differ; })
		<< "a built stamp must be comparable with a message-derived one";
}

TEST(MsgConversion, timestampZeroRoundTrip)
{
	EXPECT_EQ(timestampFromROS(timestampToROS(0.0)), 0.0);
}

/////////////////////////
// sizeOfPointField
/////////////////////////

TEST(MsgConversion, sizeOfPointField)
{
	EXPECT_EQ(sizeOfPointField(sensor_msgs::msg::PointField::INT8), 1);
	EXPECT_EQ(sizeOfPointField(sensor_msgs::msg::PointField::UINT8), 1);
	EXPECT_EQ(sizeOfPointField(sensor_msgs::msg::PointField::INT16), 2);
	EXPECT_EQ(sizeOfPointField(sensor_msgs::msg::PointField::UINT16), 2);
	EXPECT_EQ(sizeOfPointField(sensor_msgs::msg::PointField::INT32), 4);
	EXPECT_EQ(sizeOfPointField(sensor_msgs::msg::PointField::UINT32), 4);
	EXPECT_EQ(sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32), 4);
	EXPECT_EQ(sizeOfPointField(sensor_msgs::msg::PointField::FLOAT64), 8);
}

TEST(MsgConversion, sizeOfPointFieldThrowsOnUnknownType)
{
	EXPECT_THROW(sizeOfPointField(42), std::runtime_error);
}

/////////////////////////
// getClosestIterator
/////////////////////////

TEST(MsgConversion, getClosestIterator)
{
	const std::map<double, int> buffer = {{1.0, 10}, {2.0, 20}, {3.0, 30}};

	EXPECT_EQ(getClosestIterator(buffer, 1.0)->second, 10) << "exact match";
	EXPECT_EQ(getClosestIterator(buffer, 2.0)->second, 20) << "exact match";
	EXPECT_EQ(getClosestIterator(buffer, 1.4)->second, 10) << "closer to lower";
	EXPECT_EQ(getClosestIterator(buffer, 1.6)->second, 20) << "closer to upper";
	EXPECT_EQ(getClosestIterator(buffer, 0.0)->second, 10) << "clamped below range";
	EXPECT_EQ(getClosestIterator(buffer, 99.0)->second, 30) << "clamped above range";
}

TEST(MsgConversion, getClosestIteratorSingleEntry)
{
	const std::map<double, int> buffer = {{5.0, 50}};

	EXPECT_EQ(getClosestIterator(buffer, 0.0)->second, 50);
	EXPECT_EQ(getClosestIterator(buffer, 99.0)->second, 50);
}

/////////////////////////
// compressedMat <-> bytes
/////////////////////////

TEST(MsgConversion, compressedMatRoundTrip)
{
	const cv::Mat in = (cv::Mat_<unsigned char>(1, 5) << 1, 2, 3, 250, 255);

	std::vector<unsigned char> bytes;
	compressedMatToBytes(in, bytes);
	ASSERT_EQ(bytes.size(), 5u);

	const cv::Mat out = compressedMatFromBytes(bytes);
	ASSERT_EQ(out.type(), CV_8UC1);
	ASSERT_EQ(out.total(), in.total());
	EXPECT_EQ(cv::countNonZero(out.reshape(1, 1) != in.reshape(1, 1)), 0);
}

TEST(MsgConversion, compressedMatEmptyRoundTrip)
{
	std::vector<unsigned char> bytes = {1, 2, 3};
	compressedMatToBytes(cv::Mat(), bytes);

	EXPECT_TRUE(bytes.empty()) << "output must be cleared";
	EXPECT_TRUE(compressedMatFromBytes(bytes).empty());
}

TEST(MsgConversion, compressedMatFromBytesCopyFlag)
{
	std::vector<unsigned char> bytes = {1, 2, 3};

	const cv::Mat shared = compressedMatFromBytes(bytes, /*copy=*/false);
	const cv::Mat copied = compressedMatFromBytes(bytes, /*copy=*/true);

	bytes[0] = 99;
	EXPECT_EQ(shared.at<unsigned char>(0, 0), 99) << "copy=false must alias the input";
	EXPECT_EQ(copied.at<unsigned char>(0, 0), 1) << "copy=true must be independent";
}

/////////////////////////
// EnvSensor
/////////////////////////

TEST(MsgConversion, envSensorRoundTrip)
{
	const rtabmap::EnvSensor in(
			rtabmap::EnvSensor::kAmbientTemperature, 21.5, 1234567890.5);

	rtabmap_msgs::msg::EnvSensor msg;
	envSensorToROS(in, msg);
	const rtabmap::EnvSensor out = envSensorFromROS(msg);

	EXPECT_EQ(out.type(), in.type());
	EXPECT_DOUBLE_EQ(out.value(), in.value());
	EXPECT_NEAR(out.stamp(), in.stamp(), 1e-6);
}

TEST(MsgConversion, envSensorsRoundTripKeyedByType)
{
	rtabmap::EnvSensors in;
	in.insert(std::make_pair(
			rtabmap::EnvSensor::kAmbientTemperature,
			rtabmap::EnvSensor(rtabmap::EnvSensor::kAmbientTemperature, 21.5, 1.0)));
	in.insert(std::make_pair(
			rtabmap::EnvSensor::kAmbientLight,
			rtabmap::EnvSensor(rtabmap::EnvSensor::kAmbientLight, 300.0, 2.0)));

	std::vector<rtabmap_msgs::msg::EnvSensor> msg;
	envSensorsToROS(in, msg);
	ASSERT_EQ(msg.size(), in.size());

	const rtabmap::EnvSensors out = envSensorsFromROS(msg);
	ASSERT_EQ(out.size(), in.size());
	for(rtabmap::EnvSensors::const_iterator iter=in.begin(); iter!=in.end(); ++iter)
	{
		rtabmap::EnvSensors::const_iterator found = out.find(iter->first);
		ASSERT_NE(found, out.end()) << "missing type " << iter->first;
		EXPECT_DOUBLE_EQ(found->second.value(), iter->second.value());
	}
}

/////////////////////////
// Link
/////////////////////////

TEST(MsgConversion, linkRoundTrip)
{
	cv::Mat information = cv::Mat::eye(6, 6, CV_64FC1) * 3.0;
	const rtabmap::Link in(
			1, 2, rtabmap::Link::kGlobalClosure, sampleTransform(), information);

	rtabmap_msgs::msg::Link msg;
	linkToROS(in, msg);
	const rtabmap::Link out = linkFromROS(msg);

	EXPECT_EQ(out.from(), in.from());
	EXPECT_EQ(out.to(), in.to());
	EXPECT_EQ(out.type(), in.type());
	expectTransformNear(out.transform(), in.transform());

	ASSERT_EQ(out.infMatrix().rows, 6);
	ASSERT_EQ(out.infMatrix().cols, 6);
	for(int i=0; i<6; ++i)
	{
		for(int j=0; j<6; ++j)
		{
			EXPECT_DOUBLE_EQ(
					out.infMatrix().at<double>(i, j),
					in.infMatrix().at<double>(i, j)) << "at " << i << "," << j;
		}
	}
}

/////////////////////////
// CameraModel
/////////////////////////

TEST(MsgConversion, cameraModelFromROSReadsIntrinsics)
{
	sensor_msgs::msg::CameraInfo in;
	in.width = 640;
	in.height = 480;
	in.distortion_model = "plumb_bob";
	in.d = {0.1, 0.2, 0.3, 0.4, 0.5};
	in.k = {525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0};
	in.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	in.p = {525.0, 0.0, 320.0, 0.0, 0.0, 525.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};

	const rtabmap::Transform localTransform(0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f);
	const rtabmap::CameraModel model = cameraModelFromROS(in, localTransform);

	EXPECT_EQ(model.imageWidth(), 640);
	EXPECT_EQ(model.imageHeight(), 480);
	EXPECT_NEAR(model.fx(), 525.0, 1e-9);
	EXPECT_NEAR(model.fy(), 525.0, 1e-9);
	EXPECT_NEAR(model.cx(), 320.0, 1e-9);
	EXPECT_NEAR(model.cy(), 240.0, 1e-9);
	expectTransformNear(model.localTransform(), localTransform);

	// The raw distortion coefficients are kept verbatim.
	ASSERT_EQ(model.D_raw().cols, 5);
	for(size_t i=0; i<in.d.size(); ++i)
	{
		EXPECT_NEAR(model.D_raw().at<double>(0, i), in.d[i], 1e-9) << "d at " << i;
	}
}

TEST(MsgConversion, cameraModelToROSRectifiedHasNoDistortion)
{
	sensor_msgs::msg::CameraInfo in;
	in.width = 640;
	in.height = 480;
	in.distortion_model = "plumb_bob";
	in.d = {0.1, 0.2, 0.3, 0.4, 0.5};
	in.k = {525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0};
	in.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	in.p = {525.0, 0.0, 320.0, 0.0, 0.0, 525.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};

	sensor_msgs::msg::CameraInfo out;
	cameraModelToROS(cameraModelFromROS(in, rtabmap::Transform::getIdentity()), out);

	EXPECT_EQ(out.width, in.width);
	EXPECT_EQ(out.height, in.height);

	// A model carrying a projection matrix describes an already-rectified image,
	// so cameraModelToROS deliberately emits zero distortion rather than echoing
	// back the raw coefficients. K and P do round-trip unchanged.
	EXPECT_EQ(out.distortion_model, "plumb_bob");
	ASSERT_EQ(out.d.size(), 5u);
	for(size_t i=0; i<out.d.size(); ++i)
	{
		EXPECT_DOUBLE_EQ(out.d[i], 0.0) << "d at " << i;
	}
	for(size_t i=0; i<in.k.size(); ++i)
	{
		EXPECT_NEAR(out.k[i], in.k[i], 1e-9) << "k at " << i;
	}
	for(size_t i=0; i<in.p.size(); ++i)
	{
		EXPECT_NEAR(out.p[i], in.p[i], 1e-9) << "p at " << i;
	}
}

TEST(MsgConversion, cameraModelFromROSPacksFisheyeDistortion)
{
	// Fisheye/equidistant models carry 4 coefficients, which rtabmap stores in a
	// 1x6 matrix at positions 0, 1, 4 and 5.
	for(const std::string & model : {"fisheye", "equidistant", "Kannala Brandt4"})
	{
		sensor_msgs::msg::CameraInfo in;
		in.width = 640;
		in.height = 480;
		in.distortion_model = model;
		in.d = {0.1, 0.2, 0.3, 0.4};
		in.k = {525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0};

		const cv::Mat D = cameraModelFromROS(in, rtabmap::Transform::getIdentity()).D_raw();

		ASSERT_EQ(D.total(), 6u) << "distortion_model=" << model;
		EXPECT_NEAR(D.at<double>(0, 0), 0.1, 1e-9) << "distortion_model=" << model;
		EXPECT_NEAR(D.at<double>(0, 1), 0.2, 1e-9) << "distortion_model=" << model;
		EXPECT_NEAR(D.at<double>(0, 2), 0.0, 1e-9) << "distortion_model=" << model;
		EXPECT_NEAR(D.at<double>(0, 3), 0.0, 1e-9) << "distortion_model=" << model;
		EXPECT_NEAR(D.at<double>(0, 4), 0.3, 1e-9) << "distortion_model=" << model;
		EXPECT_NEAR(D.at<double>(0, 5), 0.4, 1e-9) << "distortion_model=" << model;
	}
}

TEST(MsgConversion, cameraModelToROSUnpacksFisheyeDistortion)
{
	// Built with an empty P: cameraModelToROS only reports "equidistant" for a
	// raw (unrectified) model. Note this cannot be produced by cameraModelFromROS,
	// whose P is a fixed-size array and therefore never empty.
	cv::Mat K = (cv::Mat_<double>(3, 3) <<
			525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0);
	cv::Mat D = cv::Mat::zeros(1, 6, CV_64FC1);
	D.at<double>(0, 0) = 0.1;
	D.at<double>(0, 1) = 0.2;
	D.at<double>(0, 4) = 0.3;
	D.at<double>(0, 5) = 0.4;

	const rtabmap::CameraModel model(
			"fisheye", cv::Size(640, 480), K, D, cv::Mat(), cv::Mat(),
			rtabmap::Transform::getIdentity());

	sensor_msgs::msg::CameraInfo out;
	cameraModelToROS(model, out);

	EXPECT_EQ(out.distortion_model, "equidistant");
	ASSERT_EQ(out.d.size(), 4u);
	EXPECT_NEAR(out.d[0], 0.1, 1e-9);
	EXPECT_NEAR(out.d[1], 0.2, 1e-9);
	EXPECT_NEAR(out.d[2], 0.3, 1e-9);
	EXPECT_NEAR(out.d[3], 0.4, 1e-9);
}

TEST(MsgConversion, cameraModelToROSRationalPolynomialDistortion)
{
	cv::Mat K = (cv::Mat_<double>(3, 3) <<
			525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0);
	cv::Mat D = (cv::Mat_<double>(1, 8) <<
			0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8);

	const rtabmap::CameraModel model(
			"rational", cv::Size(640, 480), K, D, cv::Mat(), cv::Mat(),
			rtabmap::Transform::getIdentity());

	sensor_msgs::msg::CameraInfo out;
	cameraModelToROS(model, out);

	EXPECT_EQ(out.distortion_model, "rational_polynomial");
	ASSERT_EQ(out.d.size(), 8u);
	for(size_t i=0; i<out.d.size(); ++i)
	{
		EXPECT_NEAR(out.d[i], 0.1 * (i + 1), 1e-9) << "d at " << i;
	}
}

TEST(MsgConversion, cameraModelFromROSTreatsUnsetMatricesAsAbsent)
{
	// k, r and p are fixed-size arrays, so "unset" means all-zero rather than empty.
	sensor_msgs::msg::CameraInfo in;
	in.width = 640;
	in.height = 480;
	in.distortion_model = "plumb_bob";
	in.d = {0.1, 0.2, 0.3, 0.4, 0.5};
	in.k = {525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0};
	// r and p deliberately left at their all-zero defaults.

	const rtabmap::CameraModel model =
			cameraModelFromROS(in, rtabmap::Transform::getIdentity());

	EXPECT_TRUE(model.P().empty()) << "an all-zero P must not become a 3x4 of zeros";
	EXPECT_TRUE(model.R().empty()) << "an all-zero R must not become a 3x3 of zeros";
	EXPECT_FALSE(model.K_raw().empty());

	// With no P, the intrinsics must come from K instead of an all-zero P.
	EXPECT_NEAR(model.fx(), 525.0, 1e-9);
	EXPECT_NEAR(model.fy(), 525.0, 1e-9);
	EXPECT_NEAR(model.cx(), 320.0, 1e-9);
	EXPECT_NEAR(model.cy(), 240.0, 1e-9);
}

TEST(MsgConversion, cameraModelUnrectifiedFisheyeRoundTrip)
{
	// Without a projection matrix the model stays raw, so the fisheye coefficients
	// survive the full message -> model -> message round trip.
	sensor_msgs::msg::CameraInfo in;
	in.width = 640;
	in.height = 480;
	in.distortion_model = "equidistant";
	in.d = {0.1, 0.2, 0.3, 0.4};
	in.k = {525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0};

	sensor_msgs::msg::CameraInfo out;
	cameraModelToROS(cameraModelFromROS(in, rtabmap::Transform::getIdentity()), out);

	EXPECT_EQ(out.distortion_model, "equidistant");
	ASSERT_EQ(out.d.size(), 4u);
	for(size_t i=0; i<in.d.size(); ++i)
	{
		EXPECT_NEAR(out.d[i], in.d[i], 1e-9) << "d at " << i;
	}
	for(size_t i=0; i<in.k.size(); ++i)
	{
		EXPECT_NEAR(out.k[i], in.k[i], 1e-9) << "k at " << i;
	}

	// An unset R is reported as identity.
	const std::array<double, 9> identity = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
	for(size_t i=0; i<identity.size(); ++i)
	{
		EXPECT_NEAR(out.r[i], identity[i], 1e-9) << "r at " << i;
	}
}

TEST(MsgConversion, cameraModelToROSSynthesizesPFromK)
{
	// With no P of its own, cameraModelToROS builds P = [K | 0].
	cv::Mat K = (cv::Mat_<double>(3, 3) <<
			525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0);

	const rtabmap::CameraModel model(
			"raw", cv::Size(640, 480), K, cv::Mat(), cv::Mat(), cv::Mat(),
			rtabmap::Transform::getIdentity());

	sensor_msgs::msg::CameraInfo out;
	cameraModelToROS(model, out);

	const std::array<double, 12> expected = {
			525.0, 0.0, 320.0, 0.0,
			0.0, 525.0, 240.0, 0.0,
			0.0, 0.0, 1.0, 0.0};
	for(size_t i=0; i<expected.size(); ++i)
	{
		EXPECT_NEAR(out.p[i], expected[i], 1e-9) << "p at " << i;
	}
	// P(2,3) is a translation term and must stay zero for a single camera.
	EXPECT_DOUBLE_EQ(out.p[11], 0.0);
}

/////////////////////////
// GlobalDescriptor
/////////////////////////

TEST(MsgConversion, globalDescriptorRoundTrip)
{
	cv::Mat data = (cv::Mat_<float>(1, 4) << 1.0f, 2.0f, 3.0f, 4.0f);
	cv::Mat info = (cv::Mat_<float>(1, 2) << 9.0f, 8.0f);
	const rtabmap::GlobalDescriptor in(7, data, info);

	rtabmap_msgs::msg::GlobalDescriptor msg;
	globalDescriptorToROS(in, msg);
	const rtabmap::GlobalDescriptor out = globalDescriptorFromROS(msg);

	EXPECT_EQ(out.type(), in.type());
	ASSERT_EQ(out.data().total(), in.data().total());
	for(size_t i=0; i<in.data().total(); ++i)
	{
		EXPECT_FLOAT_EQ(out.data().at<float>(0, i), in.data().at<float>(0, i)) << "data at " << i;
	}
	ASSERT_EQ(out.info().total(), in.info().total());
	for(size_t i=0; i<in.info().total(); ++i)
	{
		EXPECT_FLOAT_EQ(out.info().at<float>(0, i), in.info().at<float>(0, i)) << "info at " << i;
	}
}

TEST(MsgConversion, globalDescriptorsVectorRoundTrip)
{
	std::vector<rtabmap::GlobalDescriptor> in;
	in.push_back(rtabmap::GlobalDescriptor(1, (cv::Mat_<float>(1, 2) << 1.0f, 2.0f)));
	in.push_back(rtabmap::GlobalDescriptor(2, (cv::Mat_<float>(1, 2) << 3.0f, 4.0f)));

	std::vector<rtabmap_msgs::msg::GlobalDescriptor> msg;
	globalDescriptorsToROS(in, msg);
	ASSERT_EQ(msg.size(), in.size());

	const std::vector<rtabmap::GlobalDescriptor> out = globalDescriptorsFromROS(msg);
	ASSERT_EQ(out.size(), in.size());
	for(size_t i=0; i<in.size(); ++i)
	{
		EXPECT_EQ(out[i].type(), in[i].type()) << "at " << i;
		EXPECT_FLOAT_EQ(out[i].data().at<float>(0, 0), in[i].data().at<float>(0, 0)) << "at " << i;
	}
}

TEST(MsgConversion, globalDescriptorsEmptyRoundTrip)
{
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> msg(3);
	globalDescriptorsToROS(std::vector<rtabmap::GlobalDescriptor>(), msg);

	EXPECT_TRUE(msg.empty()) << "output must be cleared";
	EXPECT_TRUE(globalDescriptorsFromROS(msg).empty());
}

/////////////////////////
// UserData
/////////////////////////

TEST(MsgConversion, userDataUncompressedRoundTrip)
{
	const cv::Mat in = (cv::Mat_<int>(2, 3) << 1, 2, 3, 4, 5, 6);

	rtabmap_msgs::msg::UserData msg;
	userDataToROS(in, msg, /*compress=*/false);

	EXPECT_EQ(msg.rows, in.rows);
	EXPECT_EQ(msg.cols, in.cols);
	EXPECT_EQ(msg.type, in.type());

	const cv::Mat out = userDataFromROS(msg);
	ASSERT_EQ(out.rows, in.rows);
	ASSERT_EQ(out.cols, in.cols);
	ASSERT_EQ(out.type(), in.type());
	EXPECT_EQ(cv::countNonZero(out != in), 0);
}

TEST(MsgConversion, userDataCompressedRoundTrip)
{
	const cv::Mat in = (cv::Mat_<int>(2, 3) << 1, 2, 3, 4, 5, 6);

	rtabmap_msgs::msg::UserData msg;
	userDataToROS(in, msg, /*compress=*/true);

	// Compressed payloads travel as a 1xN byte blob.
	EXPECT_EQ(msg.rows, 1);
	EXPECT_EQ(msg.type, CV_8UC1);
	EXPECT_EQ((size_t)msg.cols, msg.data.size());

	// userDataFromROS hands back the still-compressed blob; the caller uncompresses.
	const cv::Mat blob = userDataFromROS(msg);
	ASSERT_FALSE(blob.empty());
	const cv::Mat out = rtabmap::uncompressData(blob);

	ASSERT_EQ(out.rows, in.rows);
	ASSERT_EQ(out.cols, in.cols);
	ASSERT_EQ(out.type(), in.type());
	EXPECT_EQ(cv::countNonZero(out != in), 0);
}

TEST(MsgConversion, userDataEmpty)
{
	rtabmap_msgs::msg::UserData msg;
	userDataToROS(cv::Mat(), msg, /*compress=*/false);
	EXPECT_TRUE(msg.data.empty());
	EXPECT_TRUE(userDataFromROS(msg).empty());
}

/////////////////////////
// StereoCameraModel
/////////////////////////

TEST(MsgConversion, stereoCameraModelFromROS)
{
	const double fx = 525.0;
	const double baseline = 0.12;

	sensor_msgs::msg::CameraInfo left;
	left.width = 640;
	left.height = 480;
	left.k = {fx, 0.0, 320.0, 0.0, fx, 240.0, 0.0, 0.0, 1.0};
	left.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	left.p = {fx, 0.0, 320.0, 0.0, 0.0, fx, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};

	// The right camera carries the baseline in P(0,3) = -fx * baseline.
	sensor_msgs::msg::CameraInfo right = left;
	right.p[3] = -fx * baseline;

	const rtabmap::StereoCameraModel model = stereoCameraModelFromROS(
			left, right, rtabmap::Transform::getIdentity());

	EXPECT_NEAR(model.left().fx(), fx, 1e-9);
	EXPECT_NEAR(model.right().fx(), fx, 1e-9);
	EXPECT_NEAR(model.baseline(), baseline, 1e-9);
	EXPECT_TRUE(model.isValidForProjection());
}

/////////////////////////
// OdometryInfo
/////////////////////////

TEST(MsgConversion, odomInfoRoundTrip)
{
	rtabmap::OdometryInfo in;
	in.lost = false;
	in.features = 500;
	in.localMapSize = 1000;
	in.localScanMapSize = 2000;
	in.localKeyFrames = 5;
	in.keyFrameAdded = true;
	in.timeEstimation = 0.02f;
	in.interval = 0.033;
	in.distanceTravelled = 12.5f;
	in.reg.matches = 300;
	in.reg.inliers = 250;
	in.transform = sampleTransform();

	rtabmap_msgs::msg::OdomInfo msg;
	odomInfoToROS(in, msg);
	const rtabmap::OdometryInfo out = odomInfoFromROS(msg);

	EXPECT_EQ(out.lost, in.lost);
	EXPECT_EQ(out.features, in.features);
	EXPECT_EQ(out.localMapSize, in.localMapSize);
	EXPECT_EQ(out.localScanMapSize, in.localScanMapSize);
	EXPECT_EQ(out.localKeyFrames, in.localKeyFrames);
	EXPECT_EQ(out.keyFrameAdded, in.keyFrameAdded);
	EXPECT_FLOAT_EQ(out.timeEstimation, in.timeEstimation);
	EXPECT_NEAR(out.interval, in.interval, 1e-6);
	EXPECT_FLOAT_EQ(out.distanceTravelled, in.distanceTravelled);
	EXPECT_EQ(out.reg.matches, in.reg.matches);
	EXPECT_EQ(out.reg.inliers, in.reg.inliers);
	expectTransformNear(out.transform, in.transform);
}

TEST(MsgConversion, odomInfoIgnoreDataDropsHeavyMembers)
{
	rtabmap::OdometryInfo in;
	in.features = 500;
	in.reg.inliers = 250;
	in.words.insert(std::make_pair(1, cv::KeyPoint(cv::Point2f(1, 2), 3)));
	in.localMap.insert(std::make_pair(1, cv::Point3f(1, 2, 3)));

	rtabmap_msgs::msg::OdomInfo full;
	odomInfoToROS(in, full, /*ignoreData=*/false);
	EXPECT_FALSE(full.words_keys.empty());

	rtabmap_msgs::msg::OdomInfo light;
	odomInfoToROS(in, light, /*ignoreData=*/true);
	EXPECT_TRUE(light.words_keys.empty()) << "heavy members must be dropped";

	// The scalar statistics survive either way.
	EXPECT_EQ(odomInfoFromROS(light).features, in.features);
	EXPECT_EQ(odomInfoFromROS(light).reg.inliers, in.reg.inliers);
}

TEST(MsgConversion, odomInfoToStatistics)
{
	rtabmap::OdometryInfo info;
	info.features = 400;
	info.reg.inliers = 100;
	info.reg.matches = 200;
	info.localMapSize = 1234;

	const std::map<std::string, float> stats = odomInfoToStatistics(info);

	ASSERT_TRUE(stats.find("Odometry/Features/") != stats.end());
	EXPECT_FLOAT_EQ(stats.at("Odometry/Features/"), 400.0f);
	EXPECT_FLOAT_EQ(stats.at("Odometry/Matches/"), 200.0f);
	EXPECT_FLOAT_EQ(stats.at("Odometry/Inliers/"), 100.0f);
	EXPECT_FLOAT_EQ(stats.at("Odometry/LocalMapSize/"), 1234.0f);
	// MatchesRatio is inliers/features, and must not divide by zero.
	EXPECT_FLOAT_EQ(stats.at("Odometry/MatchesRatio/"), 100.0f/400.0f);
}

TEST(MsgConversion, odomInfoToStatisticsEmptyCovariance)
{
	// RegistrationInfo does not initialize covariance, so a plain OdometryInfo has
	// an empty matrix. Reading it must not be attempted.
	rtabmap::OdometryInfo info;
	ASSERT_TRUE(info.reg.covariance.empty()) << "precondition";

	const std::map<std::string, float> stats = odomInfoToStatistics(info);

	EXPECT_TRUE(stats.find("Odometry/StdDevLin/") == stats.end())
		<< "covariance-derived stats must be omitted, not read out of bounds";
	EXPECT_TRUE(stats.find("Odometry/VarianceAng/") == stats.end());
	// The rest of the statistics are still produced.
	EXPECT_TRUE(stats.find("Odometry/Features/") != stats.end());
}

TEST(MsgConversion, odomInfoToStatisticsWithCovariance)
{
	rtabmap::OdometryInfo info;
	info.reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 4.0;

	const std::map<std::string, float> stats = odomInfoToStatistics(info);

	ASSERT_TRUE(stats.find("Odometry/VarianceLin/") != stats.end());
	EXPECT_FLOAT_EQ(stats.at("Odometry/VarianceLin/"), 4.0f);
	EXPECT_FLOAT_EQ(stats.at("Odometry/StdDevLin/"), 2.0f);
	EXPECT_FLOAT_EQ(stats.at("Odometry/VarianceAng/"), 4.0f);
	EXPECT_FLOAT_EQ(stats.at("Odometry/StdDevAng/"), 2.0f);
}

TEST(MsgConversion, odomInfoToStatisticsNoFeatures)
{
	rtabmap::OdometryInfo info;
	info.features = 0;
	info.reg.inliers = 10;

	EXPECT_FLOAT_EQ(odomInfoToStatistics(info).at("Odometry/MatchesRatio/"), 0.0f)
		<< "must not divide by zero";
}

/////////////////////////
// MapGraph / MapData
/////////////////////////

TEST(MsgConversion, mapGraphRoundTrip)
{
	std::map<int, rtabmap::Transform> poses;
	poses.insert(std::make_pair(1, rtabmap::Transform(1, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(2, sampleTransform()));

	std::multimap<int, rtabmap::Link> links;
	links.insert(std::make_pair(1, rtabmap::Link(
			1, 2, rtabmap::Link::kNeighbor, sampleTransform(),
			cv::Mat::eye(6, 6, CV_64FC1) * 2.0)));
	links.insert(std::make_pair(2, rtabmap::Link(
			2, 1, rtabmap::Link::kGlobalClosure, rtabmap::Transform::getIdentity(),
			cv::Mat::eye(6, 6, CV_64FC1))));

	const rtabmap::Transform mapToOdom(0.5f, -0.5f, 0.0f, 0.0f, 0.0f, 0.1f);

	rtabmap_msgs::msg::MapGraph msg;
	mapGraphToROS(poses, links, mapToOdom, msg);
	ASSERT_EQ(msg.poses.size(), poses.size());
	ASSERT_EQ(msg.poses_id.size(), poses.size());
	ASSERT_EQ(msg.links.size(), links.size());

	std::map<int, rtabmap::Transform> outPoses;
	std::multimap<int, rtabmap::Link> outLinks;
	rtabmap::Transform outMapToOdom;
	mapGraphFromROS(msg, outPoses, outLinks, outMapToOdom);

	ASSERT_EQ(outPoses.size(), poses.size());
	for(std::map<int, rtabmap::Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		ASSERT_TRUE(outPoses.find(iter->first) != outPoses.end()) << "missing pose " << iter->first;
		expectTransformNear(outPoses.at(iter->first), iter->second);
	}

	ASSERT_EQ(outLinks.size(), links.size());
	for(std::multimap<int, rtabmap::Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		std::multimap<int, rtabmap::Link>::const_iterator found = outLinks.find(iter->first);
		ASSERT_TRUE(found != outLinks.end()) << "missing link from " << iter->first;
		EXPECT_EQ(found->second.from(), iter->second.from());
		EXPECT_EQ(found->second.to(), iter->second.to());
		EXPECT_EQ(found->second.type(), iter->second.type());
	}

	expectTransformNear(outMapToOdom, mapToOdom);
}

TEST(MsgConversion, mapGraphEmptyRoundTrip)
{
	rtabmap_msgs::msg::MapGraph msg;
	mapGraphToROS(std::map<int, rtabmap::Transform>(), std::multimap<int, rtabmap::Link>(),
			rtabmap::Transform(), msg);

	EXPECT_TRUE(msg.poses.empty());
	EXPECT_TRUE(msg.links.empty());

	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	rtabmap::Transform mapToOdom;
	mapGraphFromROS(msg, poses, links, mapToOdom);

	EXPECT_TRUE(poses.empty());
	EXPECT_TRUE(links.empty());
	EXPECT_TRUE(mapToOdom.isNull()) << "a null map_to_odom must survive as null";
}

TEST(MsgConversion, mapDataRoundTrip)
{
	std::map<int, rtabmap::Transform> poses;
	poses.insert(std::make_pair(1, sampleTransform()));

	std::multimap<int, rtabmap::Link> links;
	links.insert(std::make_pair(1, rtabmap::Link(
			1, 2, rtabmap::Link::kNeighbor, sampleTransform())));

	std::map<int, rtabmap::Signature> signatures;
	rtabmap::Signature sig(1, 0, 3, 1234.5, "my_label", sampleTransform());
	signatures.insert(std::make_pair(1, sig));

	const rtabmap::Transform mapToOdom = rtabmap::Transform::getIdentity();

	rtabmap_msgs::msg::MapData msg;
	mapDataToROS(poses, links, signatures, mapToOdom, msg);
	ASSERT_EQ(msg.nodes.size(), signatures.size());
	ASSERT_EQ(msg.graph.poses.size(), poses.size());

	std::map<int, rtabmap::Transform> outPoses;
	std::multimap<int, rtabmap::Link> outLinks;
	std::map<int, rtabmap::Signature> outSignatures;
	rtabmap::Transform outMapToOdom;
	mapDataFromROS(msg, outPoses, outLinks, outSignatures, outMapToOdom);

	EXPECT_EQ(outPoses.size(), poses.size());
	EXPECT_EQ(outLinks.size(), links.size());
	ASSERT_EQ(outSignatures.size(), signatures.size());
	ASSERT_TRUE(outSignatures.find(1) != outSignatures.end());
	EXPECT_EQ(outSignatures.at(1).id(), sig.id());
	EXPECT_EQ(outSignatures.at(1).getLabel(), sig.getLabel());
	EXPECT_EQ(outSignatures.at(1).getWeight(), sig.getWeight());
	EXPECT_NEAR(outSignatures.at(1).getStamp(), sig.getStamp(), 1e-6);
}

/////////////////////////
// Node / Signature
/////////////////////////

namespace {

rtabmap::Signature sampleSignature()
{
	rtabmap::Signature s(7, 2, 3, 1234.5, "node_label", sampleTransform());

	std::multimap<int, int> words;
	std::vector<cv::KeyPoint> kpts;
	std::vector<cv::Point3f> pts3;
	cv::Mat descriptors(2, 4, CV_32FC1);
	for(int i=0; i<2; ++i)
	{
		words.insert(std::make_pair(100 + i, i));
		kpts.push_back(cv::KeyPoint(cv::Point2f(10.0f * i, 20.0f * i), 7.0f));
		pts3.push_back(cv::Point3f(1.0f * i, 2.0f * i, 3.0f * i));
		for(int j=0; j<4; ++j)
		{
			descriptors.at<float>(i, j) = float(i * 4 + j);
		}
	}
	s.setWords(words, kpts, pts3, descriptors);
	return s;
}

}  // namespace

TEST(MsgConversion, nodeRoundTrip)
{
	const rtabmap::Signature in = sampleSignature();

	rtabmap_msgs::msg::Node msg;
	nodeToROS(in, msg);
	const rtabmap::Signature out = nodeFromROS(msg);

	EXPECT_EQ(out.id(), in.id());
	EXPECT_EQ(out.mapId(), in.mapId());
	EXPECT_EQ(out.getWeight(), in.getWeight());
	EXPECT_NEAR(out.getStamp(), in.getStamp(), 1e-6);
	EXPECT_EQ(out.getLabel(), in.getLabel());
	expectTransformNear(out.getPose(), in.getPose());

	// Visual words: ids, keypoints, 3D points and descriptors.
	ASSERT_EQ(out.getWords().size(), in.getWords().size());
	EXPECT_TRUE(std::equal(out.getWords().begin(), out.getWords().end(), in.getWords().begin()));

	ASSERT_EQ(out.getWordsKpts().size(), in.getWordsKpts().size());
	for(size_t i=0; i<in.getWordsKpts().size(); ++i)
	{
		EXPECT_FLOAT_EQ(out.getWordsKpts()[i].pt.x, in.getWordsKpts()[i].pt.x) << "kpt " << i;
		EXPECT_FLOAT_EQ(out.getWordsKpts()[i].pt.y, in.getWordsKpts()[i].pt.y) << "kpt " << i;
	}

	ASSERT_EQ(out.getWords3().size(), in.getWords3().size());
	for(size_t i=0; i<in.getWords3().size(); ++i)
	{
		EXPECT_FLOAT_EQ(out.getWords3()[i].x, in.getWords3()[i].x) << "pt3 " << i;
		EXPECT_FLOAT_EQ(out.getWords3()[i].z, in.getWords3()[i].z) << "pt3 " << i;
	}

	ASSERT_EQ(out.getWordsDescriptors().rows, in.getWordsDescriptors().rows);
	ASSERT_EQ(out.getWordsDescriptors().cols, in.getWordsDescriptors().cols);
	EXPECT_EQ(cv::countNonZero(out.getWordsDescriptors() != in.getWordsDescriptors()), 0);
}

TEST(MsgConversion, nodeGroundTruthRoundTrip)
{
	// The ground truth travels in the Node's SensorData sub-message but is written and
	// read by the Node conversion itself.
	rtabmap::Signature in(7, 2, 3, 1234.5, "node_label",
			sampleTransform(), rtabmap::Transform(9.0f, 8.0f, 7.0f, 0.0f, 0.0f, 0.0f));

	rtabmap_msgs::msg::Node msg;
	nodeToROS(in, msg);
	const rtabmap::Signature out = nodeFromROS(msg);

	expectTransformNear(out.getGroundTruthPose(), in.getGroundTruthPose());
}

TEST(MsgConversion, nodeInfoRoundTripCarriesNoSensorData)
{
	const rtabmap::Signature in = sampleSignature();

	rtabmap_msgs::msg::Node msg;
	nodeInfoToROS(in, msg);
	const rtabmap::Signature out = nodeInfoFromROS(msg);

	EXPECT_EQ(out.id(), in.id());
	EXPECT_EQ(out.mapId(), in.mapId());
	EXPECT_EQ(out.getWeight(), in.getWeight());
	EXPECT_EQ(out.getLabel(), in.getLabel());
	expectTransformNear(out.getPose(), in.getPose());
}

TEST(MsgConversion, nodeDataRoundTripCarriesWords)
{
	const rtabmap::Signature in = sampleSignature();

	rtabmap_msgs::msg::Node msg;
	nodeDataToROS(in, msg);
	const rtabmap::Signature out = nodeDataFromROS(msg);

	EXPECT_EQ(out.getWords().size(), in.getWords().size());
	EXPECT_EQ(out.getWordsKpts().size(), in.getWordsKpts().size());
	EXPECT_EQ(out.getWords3().size(), in.getWords3().size());
}

TEST(MsgConversion, nodeEmptyRoundTrip)
{
	rtabmap::Signature in(1);

	rtabmap_msgs::msg::Node msg;
	nodeToROS(in, msg);
	const rtabmap::Signature out = nodeFromROS(msg);

	EXPECT_EQ(out.id(), 1);
	EXPECT_TRUE(out.getWords().empty());
	EXPECT_TRUE(out.getWordsKpts().empty());
	EXPECT_TRUE(out.getWordsDescriptors().empty());
}

/////////////////////////
// SensorData
/////////////////////////

TEST(MsgConversion, sensorDataRoundTrip)
{
	cv::Mat K = (cv::Mat_<double>(3, 3) <<
			525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0);
	const rtabmap::CameraModel model(
			"cam", cv::Size(640, 480), K, cv::Mat(), cv::Mat(), cv::Mat(),
			rtabmap::Transform(0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f));

	rtabmap::SensorData in(cv::Mat(), cv::Mat(), model, 42, 1234.5);
	in.setGroundTruth(sampleTransform());
	in.setGPS(rtabmap::GPS(1234.5, -71.9, 45.4, 100.0, 5.0, 90.0));

	rtabmap_msgs::msg::SensorData msg;
	sensorDataToROS(in, msg, "base_link");
	EXPECT_EQ(msg.header.frame_id, "base_link");

	const rtabmap::SensorData out = sensorDataFromROS(msg);

	EXPECT_NEAR(out.stamp(), in.stamp(), 1e-6);

	// sensorDataToROS writes ground_truth_pose into the message, but sensorDataFromROS
	// deliberately does not read it back: the ground truth is owned by the enclosing
	// Node conversion (nodeFromROS feeds it to the Signature constructor). See
	// nodeGroundTruthRoundTrip for the round trip that does preserve it.
	EXPECT_FALSE(transformFromPoseMsg(msg.ground_truth_pose).isNull())
		<< "the message must still carry the ground truth for nodeFromROS";
	EXPECT_TRUE(out.groundTruth().isNull())
		<< "sensorDataFromROS does not restore the ground truth";

	ASSERT_EQ(out.cameraModels().size(), 1u);
	EXPECT_NEAR(out.cameraModels()[0].fx(), 525.0, 1e-9);
	EXPECT_NEAR(out.cameraModels()[0].cx(), 320.0, 1e-9);
	expectTransformNear(
			out.cameraModels()[0].localTransform(), model.localTransform());

	EXPECT_NEAR(out.gps().longitude(), in.gps().longitude(), 1e-9);
	EXPECT_NEAR(out.gps().latitude(), in.gps().latitude(), 1e-9);
	EXPECT_NEAR(out.gps().altitude(), in.gps().altitude(), 1e-9);
	EXPECT_NEAR(out.gps().bearing(), in.gps().bearing(), 1e-9);
}

TEST(MsgConversion, sensorDataUserDataRoundTrip)
{
	rtabmap::SensorData in;
	in.setStamp(10.0);
	in.setUserData((cv::Mat_<int>(1, 3) << 7, 8, 9));

	rtabmap_msgs::msg::SensorData msg;
	sensorDataToROS(in, msg);
	const rtabmap::SensorData out = sensorDataFromROS(msg);

	const cv::Mat data = out.userDataRaw().empty()
			? rtabmap::uncompressData(out.userDataCompressed())
			: out.userDataRaw();
	ASSERT_FALSE(data.empty());
	ASSERT_EQ(data.cols, 3);
	EXPECT_EQ(data.at<int>(0, 0), 7);
	EXPECT_EQ(data.at<int>(0, 2), 9);
}

/////////////////////////
// Statistics / Info
/////////////////////////

TEST(MsgConversion, infoRoundTrip)
{
	rtabmap::Statistics in;
	in.setExtended(true);
	in.setRefImageId(5);
	in.setLoopClosureId(9);
	in.setProximityDetectionId(11);
	in.setStamp(1234.5);
	in.setLoopClosureTransform(sampleTransform());
	in.setWmState(std::vector<int>{1, 2, 3});

	std::map<int, float> posterior;
	posterior.insert(std::make_pair(1, 0.25f));
	posterior.insert(std::make_pair(2, 0.75f));
	in.setPosterior(posterior);

	std::map<int, int> weights;
	weights.insert(std::make_pair(1, 10));
	in.setWeights(weights);

	std::map<int, std::string> labels;
	labels.insert(std::make_pair(1, "kitchen"));
	in.setLabels(labels);

	in.addStatistic("Some/Stat/", 3.5f);

	rtabmap_msgs::msg::Info msg;
	infoToROS(in, msg);

	// infoToROS leaves the header to the caller (see CoreWrapper, which stamps the
	// message before calling it), so infoFromROS can only recover the stamp if the
	// header was filled in the same way.
	EXPECT_EQ(msg.header.stamp.sec, 0) << "infoToROS must not touch the header";
	msg.header.stamp = timestampToROS(in.stamp());

	rtabmap::Statistics out;
	infoFromROS(msg, out);

	EXPECT_EQ(out.refImageId(), in.refImageId());
	EXPECT_EQ(out.loopClosureId(), in.loopClosureId());
	EXPECT_EQ(out.proximityDetectionId(), in.proximityDetectionId());
	EXPECT_NEAR(out.stamp(), in.stamp(), 1e-6);
	expectTransformNear(out.loopClosureTransform(), in.loopClosureTransform());
	EXPECT_EQ(out.wmState(), in.wmState());

	ASSERT_EQ(out.posterior().size(), in.posterior().size());
	EXPECT_FLOAT_EQ(out.posterior().at(1), 0.25f);
	EXPECT_FLOAT_EQ(out.posterior().at(2), 0.75f);

	ASSERT_EQ(out.weights().size(), in.weights().size());
	EXPECT_EQ(out.weights().at(1), 10);

	ASSERT_EQ(out.labels().size(), in.labels().size());
	EXPECT_EQ(out.labels().at(1), "kitchen");

	ASSERT_TRUE(out.data().find("Some/Stat/") != out.data().end());
	EXPECT_FLOAT_EQ(out.data().at("Some/Stat/"), 3.5f);
}

/////////////////////////
// PointCloud2 helpers
/////////////////////////

namespace {

/// Builds a dense, unorganized XYZ float cloud from the given points.
sensor_msgs::msg::PointCloud2 makeXYZCloud(const std::vector<cv::Point3f> & points)
{
	sensor_msgs::msg::PointCloud2 cloud;
	cloud.height = 1;
	cloud.width = points.size();
	cloud.is_bigendian = false;
	cloud.is_dense = true;
	cloud.fields.resize(3);
	const char * names[3] = {"x", "y", "z"};
	for(int i=0; i<3; ++i)
	{
		cloud.fields[i].name = names[i];
		cloud.fields[i].offset = 4 * i;
		cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
		cloud.fields[i].count = 1;
	}
	cloud.point_step = 12;
	cloud.row_step = cloud.point_step * cloud.width;
	cloud.data.resize(cloud.row_step * cloud.height);
	for(size_t i=0; i<points.size(); ++i)
	{
		float * p = reinterpret_cast<float *>(&cloud.data[i * cloud.point_step]);
		p[0] = points[i].x;
		p[1] = points[i].y;
		p[2] = points[i].z;
	}
	return cloud;
}

cv::Point3f readXYZ(const sensor_msgs::msg::PointCloud2 & cloud, size_t index)
{
	const float * p = reinterpret_cast<const float *>(&cloud.data[index * cloud.point_step]);
	return cv::Point3f(p[0], p[1], p[2]);
}

}  // namespace

TEST(MsgConversion, transformPointCloudTranslation)
{
	const std::vector<cv::Point3f> points = {{1.0f, 2.0f, 3.0f}, {-1.0f, 0.0f, 1.0f}};
	const sensor_msgs::msg::PointCloud2 in = makeXYZCloud(points);

	Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
	t(0, 3) = 10.0f;
	t(1, 3) = 20.0f;
	t(2, 3) = 30.0f;

	sensor_msgs::msg::PointCloud2 out;
	transformPointCloud(t, in, out);

	ASSERT_EQ(out.width, in.width);
	ASSERT_EQ(out.point_step, in.point_step);
	for(size_t i=0; i<points.size(); ++i)
	{
		const cv::Point3f p = readXYZ(out, i);
		EXPECT_NEAR(p.x, points[i].x + 10.0f, 1e-4) << "point " << i;
		EXPECT_NEAR(p.y, points[i].y + 20.0f, 1e-4) << "point " << i;
		EXPECT_NEAR(p.z, points[i].z + 30.0f, 1e-4) << "point " << i;
	}
}

TEST(MsgConversion, transformPointCloudRotation)
{
	// 90 degrees about z maps (1,0,0) to (0,1,0).
	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}};
	const sensor_msgs::msg::PointCloud2 in = makeXYZCloud(points);

	const Eigen::Matrix4f t =
			rtabmap::Transform(0, 0, 0, 0, 0, M_PI/2.0).toEigen4f();

	sensor_msgs::msg::PointCloud2 out;
	transformPointCloud(t, in, out);

	const cv::Point3f p = readXYZ(out, 0);
	EXPECT_NEAR(p.x, 0.0f, 1e-5);
	EXPECT_NEAR(p.y, 1.0f, 1e-5);
	EXPECT_NEAR(p.z, 0.0f, 1e-5);
}

TEST(MsgConversion, transformPointCloudIdentityPreservesMetadata)
{
	const sensor_msgs::msg::PointCloud2 in = makeXYZCloud({{1.0f, 2.0f, 3.0f}});

	sensor_msgs::msg::PointCloud2 out;
	transformPointCloud(Eigen::Matrix4f::Identity(), in, out);

	EXPECT_EQ(out.height, in.height);
	EXPECT_EQ(out.width, in.width);
	EXPECT_EQ(out.point_step, in.point_step);
	EXPECT_EQ(out.row_step, in.row_step);
	EXPECT_EQ(out.is_dense, in.is_dense);
	ASSERT_EQ(out.fields.size(), in.fields.size());
	for(size_t i=0; i<in.fields.size(); ++i)
	{
		EXPECT_EQ(out.fields[i].name, in.fields[i].name) << "field " << i;
	}

	const cv::Point3f p = readXYZ(out, 0);
	EXPECT_NEAR(p.x, 1.0f, 1e-5);
	EXPECT_NEAR(p.y, 2.0f, 1e-5);
	EXPECT_NEAR(p.z, 3.0f, 1e-5);
}

namespace {

// A robot driving straight at a wall while the lidar sweeps.
// A 10 Hz lidar: 100 samples 1 ms apart, so the sweep spans 99 ms and scans repeat
// every 100 ms. Scan N covers [1000.000, 1000.099], scan N+1 covers [1000.100, 1000.199].
constexpr size_t kScanPoints = 100;      // samples per sweep
constexpr double kScanStep = 0.001;      // s between consecutive samples
constexpr double kScanSpan = kScanStep * (kScanPoints - 1);   // 0.099 s, first -> last
constexpr float kWallDistance = 5.0f;    // m, distance to the wall at the first point
constexpr float kSpeed = 1.0f;           // m/s forward (+x)

/// How the per-point time channel is encoded. deskew() accepts three datatypes, and
/// FLOAT64 differs from the other two: it carries ABSOLUTE stamps (with an automatic
/// ms/us/ns unit guess), while UINT32 and FLOAT32 carry offsets from the header stamp.
enum TimeEncoding
{
	kOffsetSecFloat32,   ///< FLOAT32 seconds, relative to header.stamp
	kOffsetNsecUint32,   ///< UINT32 nanoseconds, relative to header.stamp
	kAbsoluteSecFloat64, ///< FLOAT64 absolute seconds
	kAbsoluteMsecFloat64 ///< FLOAT64 absolute milliseconds (auto-scaled by deskew)
};

/// Organized-cloud layout. deskew() picks its traversal from width>height, so the two
/// orderings exercise different loops.
enum ScanLayout
{
	kTimeOnColumns,  ///< Ouster style: width=time samples, height=rings
	kTimeOnRows      ///< Velodyne style: height=time samples, width=rings
};

/**
 * Builds the raw (skewed) scan of a flat wall captured while moving forward.
 *
 * Each time sample is taken 1 ms after the previous one, by which time the robot has
 * closed in on the wall by kSpeed * elapsed. Expressed in the sensor frame at capture
 * time, the wall therefore appears to slide towards the robot: a straight wall is
 * recorded as a slanted line. Deskewing must undo exactly that.
 *
 * @param headerStamp       absolute stamp put in the message header
 * @param firstPointOffset  time of the first sample relative to the header stamp
 * @param encoding          how to write the time channel
 * @param layout            whether time runs along columns or rows
 * @param rings             number of rings (the non-time dimension)
 * @param fieldName         name of the time channel
 * @param descendingTime    emit the samples newest-first, which deskew has to detect
 * @param displacement      distance travelled as a function of time since the first
 *                          sample; defaults to the constant-velocity kSpeed * elapsed
 */
sensor_msgs::msg::PointCloud2 makeSkewedWallScan(
		double headerStamp,
		double firstPointOffset,
		TimeEncoding encoding = kOffsetSecFloat32,
		ScanLayout layout = kTimeOnColumns,
		size_t rings = 1,
		const std::string & fieldName = "t",
		bool descendingTime = false,
		const std::function<double(double)> & displacement = nullptr)
{
	const bool timeIs64Bit =
			encoding == kAbsoluteSecFloat64 || encoding == kAbsoluteMsecFloat64;
	// Keep the 8-byte time channel aligned: x,y,z then 4 bytes of padding.
	const uint32_t timeOffset = timeIs64Bit ? 16 : 12;
	const uint32_t pointStep = timeIs64Bit ? 24 : 16;

	sensor_msgs::msg::PointCloud2 cloud;
	cloud.header.stamp = timestampToROS(headerStamp);
	cloud.header.frame_id = "base_link";
	cloud.is_bigendian = false;
	cloud.is_dense = true;
	if(layout == kTimeOnColumns)
	{
		cloud.width = kScanPoints;
		cloud.height = rings;
	}
	else
	{
		cloud.width = rings;
		cloud.height = kScanPoints;
	}

	cloud.fields.resize(4);
	const char * xyz[3] = {"x", "y", "z"};
	for(int i=0; i<3; ++i)
	{
		cloud.fields[i].name = xyz[i];
		cloud.fields[i].offset = 4 * i;
		cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
		cloud.fields[i].count = 1;
	}
	cloud.fields[3].name = fieldName;
	cloud.fields[3].offset = timeOffset;
	cloud.fields[3].datatype =
			encoding == kOffsetNsecUint32 ? sensor_msgs::msg::PointField::UINT32 :
			timeIs64Bit                   ? sensor_msgs::msg::PointField::FLOAT64 :
			                                sensor_msgs::msg::PointField::FLOAT32;
	cloud.fields[3].count = 1;

	cloud.point_step = pointStep;
	cloud.row_step = cloud.point_step * cloud.width;
	cloud.data.resize(cloud.row_step * cloud.height);

	for(size_t i=0; i<kScanPoints; ++i)          // position in the message
	{
		// When descending, the point stored first is the one captured last.
		const size_t sample = descendingTime ? (kScanPoints - 1 - i) : i;
		const double elapsed = double(sample) * kScanStep;    // since the first capture
		const double offset = firstPointOffset + elapsed;     // relative to the header
		const double absolute = headerStamp + offset;

		for(size_t r=0; r<rings; ++r)
		{
			const size_t row = (layout == kTimeOnColumns) ? r : i;
			const size_t col = (layout == kTimeOnColumns) ? i : r;
			unsigned char * base = &cloud.data[row * cloud.row_step + col * cloud.point_step];

			float * p = reinterpret_cast<float *>(base);
			// The robot has closed in on the wall by this much when the sample was taken.
			const double travelled = displacement ? displacement(elapsed) : kSpeed * elapsed;
			p[0] = kWallDistance - float(travelled);          // the skew
			p[1] = -1.0f + 2.0f * float(sample) / float(kScanPoints - 1);
			p[2] = 0.1f * float(r);                            // one plane per ring

			switch(encoding)
			{
				case kOffsetSecFloat32:
					*reinterpret_cast<float *>(base + timeOffset) = float(offset);
					break;
				case kOffsetNsecUint32:
					*reinterpret_cast<uint32_t *>(base + timeOffset) =
							uint32_t(std::llround(offset * 1e9));
					break;
				case kAbsoluteSecFloat64:
					*reinterpret_cast<double *>(base + timeOffset) = absolute;
					break;
				case kAbsoluteMsecFloat64:
					*reinterpret_cast<double *>(base + timeOffset) = absolute * 1e3;
					break;
			}
		}
	}
	return cloud;
}

/// Reads x of the point at (time sample, ring) for the given layout.
float readWallX(const sensor_msgs::msg::PointCloud2 & cloud, size_t sample, size_t ring,
		ScanLayout layout)
{
	const size_t row = (layout == kTimeOnColumns) ? ring : sample;
	const size_t col = (layout == kTimeOnColumns) ? sample : ring;
	return *reinterpret_cast<const float *>(
			&cloud.data[row * cloud.row_step + col * cloud.point_step]);
}

float readField(const sensor_msgs::msg::PointCloud2 & cloud, size_t index, size_t field)
{
	return *reinterpret_cast<const float *>(
			&cloud.data[index * cloud.point_step + cloud.fields[field].offset]);
}

}  // namespace

TEST(MsgConversion, deskewConstantVelocityHeaderAtFirstPoint)
{
	const double firstPointStamp = 1000.0;

	// Header stamped at the first point, so "t" runs 0 .. +0.100 s.
	const sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(firstPointStamp, 0.0);
	ASSERT_NEAR(readField(in, 0, 0), kWallDistance, 1e-4) << "first point is unskewed";
	ASSERT_NEAR(readField(in, kScanPoints-1, 0), kWallDistance - float(kScanSpan), 1e-4)
		<< "last point is skewed by v*0.099s = 9.9 cm";

	sensor_msgs::msg::PointCloud2 out;
	ASSERT_TRUE(deskew(in, out, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));

	// Everything collapses back onto the wall at its original distance.
	for(size_t i=0; i<kScanPoints; ++i)
	{
		EXPECT_NEAR(readField(out, i, 0), kWallDistance, 1e-4) << "x of point " << i;
		EXPECT_NEAR(readField(out, i, 1), readField(in, i, 1), 1e-6) << "y of point " << i;
		EXPECT_NEAR(readField(out, i, 2), 0.0f, 1e-6) << "z of point " << i;
		EXPECT_FLOAT_EQ(readField(out, i, 3), 0.0f)
			<< "t must be zeroed to mark the cloud as deskewed, point " << i;
	}
}

TEST(MsgConversion, deskewConstantVelocityHeaderAtLastPoint)
{
	const double firstPointStamp = 1000.0;
	const double lastPointStamp = firstPointStamp + kScanSpan;

	// Same physical scan, but stamped at the last point: "t" runs -0.100 .. 0 s.
	const sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(lastPointStamp, -kScanSpan);

	sensor_msgs::msg::PointCloud2 out;
	ASSERT_TRUE(deskew(in, out, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));

	// Still a straight line, but now expressed in the frame at the END of the scan,
	// by which point the robot has advanced kSpeed * kScanSpan = 9.9 cm.
	const float expected = kWallDistance - kSpeed * float(kScanSpan);
	for(size_t i=0; i<kScanPoints; ++i)
	{
		EXPECT_NEAR(readField(out, i, 0), expected, 1e-4) << "x of point " << i;
	}

	// The line is flat to well under a millimetre: that is the deskewing working,
	// independently of which end of the scan the frame is anchored to.
	float minX = readField(out, 0, 0);
	float maxX = minX;
	for(size_t i=1; i<kScanPoints; ++i)
	{
		minX = std::min(minX, readField(out, i, 0));
		maxX = std::max(maxX, readField(out, i, 0));
	}
	EXPECT_LT(maxX - minX, 1e-3f) << "deskewed scan must be flat";
}

TEST(MsgConversion, deskewConstantVelocityStationaryRobotIsNoOp)
{
	// With no motion there is nothing to correct, so the skewed input must come back
	// unchanged -- this pins that the correction is driven by the velocity and not by
	// something incidental to the time field.
	const double firstPointStamp = 1000.0;
	const sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(firstPointStamp, 0.0);

	sensor_msgs::msg::PointCloud2 out;
	ASSERT_TRUE(deskew(in, out, rtabmap::Transform(0, 0, 0, 0, 0, 0)));

	for(size_t i=0; i<kScanPoints; ++i)
	{
		EXPECT_NEAR(readField(out, i, 0), readField(in, i, 0), 1e-6) << "point " << i;
	}
}

/// Deskews a wall scan built with the given encoding/layout and asserts it comes out
/// flat at the expected distance.
void expectDeskewRecoversWall(
		TimeEncoding encoding,
		ScanLayout layout,
		size_t rings,
		double headerStamp,
		const std::string & fieldName = "t")
{
	SCOPED_TRACE("encoding=" + std::to_string(int(encoding)) +
			" layout=" + std::to_string(int(layout)) +
			" rings=" + std::to_string(rings) +
			" field=" + fieldName);

	const sensor_msgs::msg::PointCloud2 in =
			makeSkewedWallScan(headerStamp, 0.0, encoding, layout, rings, fieldName);

	sensor_msgs::msg::PointCloud2 out;
	ASSERT_TRUE(deskew(in, out, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));

	for(size_t i=0; i<kScanPoints; ++i)
	{
		for(size_t r=0; r<rings; ++r)
		{
			EXPECT_NEAR(readWallX(out, i, r, layout), kWallDistance, 1e-3)
				<< "sample " << i << " ring " << r;
		}
	}
}

TEST(MsgConversion, deskewTimeAsFloat32SecondsOffset)
{
	expectDeskewRecoversWall(kOffsetSecFloat32, kTimeOnColumns, 1, 1000.0);
}

TEST(MsgConversion, deskewTimeAsUint32NanosecondsOffset)
{
	// UINT32 offsets are unsigned, so the header can only sit at or before the scan.
	expectDeskewRecoversWall(kOffsetNsecUint32, kTimeOnColumns, 1, 1000.0);
}

TEST(MsgConversion, deskewTimeAsFloat64AbsoluteSeconds)
{
	// FLOAT64 carries absolute stamps rather than offsets.
	expectDeskewRecoversWall(kAbsoluteSecFloat64, kTimeOnColumns, 1, 1000.0);
}

TEST(MsgConversion, deskewTimeAsFloat64AbsoluteMilliseconds)
{
	// Above 1e12 deskew treats FLOAT64 stamps as milliseconds and rescales them, so
	// this needs a realistic epoch: 1.7e9 s is 1.7e12 ms.
	expectDeskewRecoversWall(kAbsoluteMsecFloat64, kTimeOnColumns, 1, 1.7e9);
}

TEST(MsgConversion, deskewTimeOnColumnsWithMultipleRings)
{
	// Ouster layout: width=101 samples > height=4 rings.
	expectDeskewRecoversWall(kOffsetSecFloat32, kTimeOnColumns, 4, 1000.0);
}

TEST(MsgConversion, deskewTimeOnRowsWithMultipleRings)
{
	// Velodyne layout: height=101 samples > width=4 rings, which takes the other loop.
	expectDeskewRecoversWall(kOffsetSecFloat32, kTimeOnRows, 4, 1000.0);
}

TEST(MsgConversion, deskewLayoutsAgree)
{
	// The same scan expressed in either layout must deskew to the same geometry.
	const double headerStamp = 1000.0;
	const size_t rings = 4;

	sensor_msgs::msg::PointCloud2 byColumns, byRows;
	ASSERT_TRUE(deskew(makeSkewedWallScan(headerStamp, 0.0, kOffsetSecFloat32, kTimeOnColumns, rings),
			byColumns, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));
	ASSERT_TRUE(deskew(makeSkewedWallScan(headerStamp, 0.0, kOffsetSecFloat32, kTimeOnRows, rings),
			byRows, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));

	for(size_t i=0; i<kScanPoints; ++i)
	{
		for(size_t r=0; r<rings; ++r)
		{
			EXPECT_NEAR(readWallX(byColumns, i, r, kTimeOnColumns),
					readWallX(byRows, i, r, kTimeOnRows), 1e-6)
				<< "sample " << i << " ring " << r;
		}
	}
}

TEST(MsgConversion, deskewAcceptsEveryTimeFieldName)
{
	for(const std::string & name : {"t", "time", "stamps", "timestamp"})
	{
		expectDeskewRecoversWall(kOffsetSecFloat32, kTimeOnColumns, 1, 1000.0, name);
	}
}

TEST(MsgConversion, deskewHandlesDescendingTimestamps)
{
	// Some drivers emit the sweep newest-first. deskew detects that the channel is not
	// ascending and rescans it for the true min/max before interpolating.
	const double headerStamp = 1000.0;
	const sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(
			headerStamp, 0.0, kOffsetSecFloat32, kTimeOnColumns, 1, "t",
			/*descendingTime=*/true);

	// Sanity: the stored order really is newest-first.
	ASSERT_NEAR(readWallX(in, 0, 0, kTimeOnColumns), kWallDistance - float(kScanSpan), 1e-4);
	ASSERT_NEAR(readWallX(in, kScanPoints-1, 0, kTimeOnColumns), kWallDistance, 1e-4);

	sensor_msgs::msg::PointCloud2 out;
	ASSERT_TRUE(deskew(in, out, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));

	for(size_t i=0; i<kScanPoints; ++i)
	{
		EXPECT_NEAR(readWallX(out, i, 0, kTimeOnColumns), kWallDistance, 1e-3)
			<< "sample " << i;
	}
}

TEST(MsgConversion, deskewRejectsUnknownTimeFieldName)
{
	const sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(
			1000.0, 0.0, kOffsetSecFloat32, kTimeOnColumns, 1, "elapsed");

	sensor_msgs::msg::PointCloud2 out;
	EXPECT_FALSE(deskew(in, out, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));
}

TEST(MsgConversion, deskewRejectsUnsupportedTimeDatatype)
{
	sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(1000.0, 0.0);
	in.fields[3].datatype = sensor_msgs::msg::PointField::INT32;   // 4 bytes, but not 6/7/8

	sensor_msgs::msg::PointCloud2 out;
	EXPECT_FALSE(deskew(in, out, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));
}

TEST(MsgConversion, deskewWithRotationIsAnchoredAtTheHeaderStamp)
{
	// A pure yaw rate makes the correction a pure rotation about z, so it can be checked
	// exactly: each sample must be rotated by yawRate * (its time - the header stamp),
	// with its distance from the origin unchanged. This is what pins the correction to
	// the header stamp -- there is no other reference time involved.
	const double headerStamp = 1000.0;
	const double yawRate = 0.5;   // rad/s
	const sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(headerStamp, 0.0);

	sensor_msgs::msg::PointCloud2 out;
	ASSERT_TRUE(deskew(in, out, rtabmap::Transform(0, 0, 0, 0, 0, yawRate)));

	auto xy = [](const sensor_msgs::msg::PointCloud2 & c, size_t i) {
		const float * p = reinterpret_cast<const float *>(&c.data[i * c.point_step]);
		return std::make_pair(p[0], p[1]);
	};

	for(size_t i=0; i<kScanPoints; ++i)
	{
		const std::pair<float, float> a = xy(in, i);
		const std::pair<float, float> b = xy(out, i);
		const double dt = double(i) * kScanStep;   // sample 0 sits at the header stamp

		EXPECT_NEAR(std::hypot(b.first, b.second), std::hypot(a.first, a.second), 1e-4)
			<< "a rotation must preserve the range of sample " << i;
		EXPECT_NEAR(std::atan2(b.second, b.first) - std::atan2(a.second, a.first),
				yawRate * dt, 1e-4)
			<< "sample " << i << " must be rotated by yawRate*dt";
	}

	// Spelling out the i=0 case: dt is zero there, so that sample is untouched.
	EXPECT_FLOAT_EQ(xy(out, 0).first, xy(in, 0).first);
	EXPECT_FLOAT_EQ(xy(out, 0).second, xy(in, 0).second);
}

TEST(MsgConversion, deskewPassesThroughWhenThereIsNoTimeSpread)
{
	// A driver that leaves the time channel at zero gives a scan with no time spread.
	// There is nothing to correct, so the cloud must come back unchanged rather than
	// being reported as a failure -- callers abort the frame on false.
	sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(1000.0, 0.0);
	for(size_t i=0; i<kScanPoints; ++i)
	{
		*reinterpret_cast<float *>(&in.data[i * in.point_step + in.fields[3].offset]) = 0.0f;
	}

	sensor_msgs::msg::PointCloud2 out;
	ASSERT_TRUE(deskew(in, out, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));
	EXPECT_EQ(out.data, in.data) << "the cloud must be returned untouched";
}

TEST(MsgConversion, deskewIsIdempotent)
{
	// Deskewing zeroes the time channel to mark the cloud as done, so running deskew a
	// second time (e.g. lidar_deskewing feeding icp_odometry) must be a silent no-op.
	const sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(1000.0, 0.0);
	const rtabmap::Transform velocity(kSpeed, 0, 0, 0, 0, 0);

	sensor_msgs::msg::PointCloud2 once;
	ASSERT_TRUE(deskew(in, once, velocity));
	for(size_t i=0; i<kScanPoints; ++i)
	{
		ASSERT_FLOAT_EQ(*reinterpret_cast<const float *>(
				&once.data[i * once.point_step + once.fields[3].offset]), 0.0f)
			<< "deskewing must zero the time channel, sample " << i;
	}

	sensor_msgs::msg::PointCloud2 twice;
	ASSERT_TRUE(deskew(once, twice, velocity)) << "a second pass must not fail";
	EXPECT_EQ(twice.data, once.data) << "a second pass must change nothing";
}

TEST(MsgConversion, deskewClampsSamplesOutsideTheSweep)
{
	// The ordering check only inspects the first and last samples, so a corrupt stamp in
	// the middle is not detected. It must be clamped to the end of the sweep rather than
	// extrapolated, which would fling the point far past the wall.
	const double headerStamp = 1000.0;
	sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(headerStamp, 0.0);
	const size_t corrupt = kScanPoints / 2;
	*reinterpret_cast<float *>(
			&in.data[corrupt * in.point_step + in.fields[3].offset]) = 0.5f;  // 5x the sweep

	sensor_msgs::msg::PointCloud2 out;
	ASSERT_TRUE(deskew(in, out, rtabmap::Transform(kSpeed, 0, 0, 0, 0, 0)));

	// Clamped to the last sample's correction, so it lands within the sweep's own range
	// rather than metres away. Every other sample is unaffected.
	const float x = readWallX(out, corrupt, 0, kTimeOnColumns);
	EXPECT_GE(x, kWallDistance - 1e-3f);
	EXPECT_LE(x, kWallDistance + float(kSpeed * kScanSpan) + 1e-3f)
		<< "an unclamped ratio of ~5 would put this point ~0.45 m past the wall";

	for(size_t i=0; i<kScanPoints; ++i)
	{
		if(i == corrupt) continue;
		EXPECT_NEAR(readWallX(out, i, 0, kTimeOnColumns), kWallDistance, 1e-3)
			<< "uncorrupted sample " << i << " must be unaffected";
	}
}

TEST(MsgConversion, deskewWithoutTimeFieldFails)
{
	// Deskewing needs a per-point time field; a plain XYZ cloud cannot be deskewed.
	const sensor_msgs::msg::PointCloud2 in = makeXYZCloud({{1.0f, 0.0f, 0.0f}});

	sensor_msgs::msg::PointCloud2 out;
	EXPECT_FALSE(deskew(in, out, rtabmap::Transform(1, 0, 0, 0, 0, 0)));
}

TEST(MsgConversion, deskewNullVelocityFails)
{
	const sensor_msgs::msg::PointCloud2 in = makeXYZCloud({{1.0f, 0.0f, 0.0f}});

	sensor_msgs::msg::PointCloud2 out;
	EXPECT_FALSE(deskew(in, out, rtabmap::Transform()))
		<< "a null velocity cannot deskew";
}

/////////////////////////
// RGBDImage
/////////////////////////

TEST(MsgConversion, rgbdImageRoundTrip)
{
	cv::Mat K = (cv::Mat_<double>(3, 3) <<
			525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0);
	const rtabmap::CameraModel model(
			"cam", cv::Size(4, 4), K, cv::Mat(), cv::Mat(), cv::Mat(),
			rtabmap::Transform(0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f));

	cv::Mat rgb(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
	cv::Mat depth(4, 4, CV_16UC1, cv::Scalar(1000));
	rtabmap::SensorData in(rgb, depth, model, 1, 1234.5);

	rtabmap_msgs::msg::RGBDImage msg;
	rgbdImageToROS(in, msg, "camera_link");

	EXPECT_EQ(msg.rgb_camera_info.header.frame_id, "camera_link");
	EXPECT_NEAR(timestampFromROS(msg.rgb_camera_info.header.stamp), 1234.5, 1e-6);

	// rgbdImageToROS stamps only the sub-messages; the top-level header is the
	// caller's job (see OdometryROS, which assigns msg.header right after the call),
	// and rgbdImageFromROS reads the stamp from that top-level header.
	EXPECT_EQ(msg.header.stamp.sec, 0) << "rgbdImageToROS must not touch the header";
	msg.header = msg.rgb_camera_info.header;

	// The returned SensorData shallow-references the message buffers, so the message
	// must outlive it -- see rgbdImageFromROSAliasesTheMessage.
	const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr held =
			std::make_shared<const rtabmap_msgs::msg::RGBDImage>(msg);
	const rtabmap::SensorData out = rgbdImageFromROS(held);

	EXPECT_NEAR(out.stamp(), in.stamp(), 1e-6);
	ASSERT_EQ(out.cameraModels().size(), 1u);
	EXPECT_NEAR(out.cameraModels()[0].fx(), 525.0, 1e-9);

	// The local transform is not carried by the message (CameraInfo has no such
	// field); callers resolve it from TF, so it comes back as the default identity.
	EXPECT_TRUE(out.cameraModels()[0].localTransform().isIdentity())
		<< out.cameraModels()[0].localTransform().prettyPrint();

	ASSERT_FALSE(out.imageRaw().empty());
	EXPECT_EQ(out.imageRaw().type(), CV_8UC3);
	EXPECT_EQ(cv::countNonZero(out.imageRaw().reshape(1) != rgb.reshape(1)), 0);

	ASSERT_FALSE(out.depthRaw().empty());
	EXPECT_EQ(out.depthRaw().type(), CV_16UC1);
	EXPECT_EQ(cv::countNonZero(out.depthRaw() != depth), 0);
}

TEST(MsgConversion, rgbdImageFromROSAliasesTheMessage)
{
	// rgbdImageFromROS deliberately avoids copying the pixels: the SensorData it returns
	// points into the message's own buffers. Mutating the message is visible through the
	// SensorData. Callers must therefore keep the message alive and unchanged for as long
	// as they use the result -- and must deep-copy before letting the SensorData outlive
	// the subscription callback, since the ROS queue recycles the message once it
	// returns.
	cv::Mat rgb(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
	cv::Mat depth(4, 4, CV_16UC1, cv::Scalar(1000));

	auto msg = std::make_shared<rtabmap_msgs::msg::RGBDImage>();
	msg->rgb_camera_info.width = 4;
	msg->rgb_camera_info.height = 4;
	msg->rgb_camera_info.k = {525.0, 0.0, 2.0, 0.0, 525.0, 2.0, 0.0, 0.0, 1.0};
	cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb).toImageMsg(msg->rgb);
	cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth).toImageMsg(msg->depth);

	const rtabmap::SensorData data = rgbdImageFromROS(msg);
	ASSERT_FALSE(data.imageRaw().empty());
	ASSERT_EQ(data.imageRaw().at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));

	// Writing through the message is observable in the SensorData: no copy was made.
	msg->rgb.data[0] = 99;
	EXPECT_EQ(data.imageRaw().at<cv::Vec3b>(0, 0)[0], 99)
		<< "SensorData is expected to alias the message buffer";
}

TEST(MsgConversion, toCvCopyReadsRawImages)
{
	cv::Mat rgb(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
	cv::Mat depth(4, 4, CV_16UC1, cv::Scalar(1000));

	rtabmap_msgs::msg::RGBDImage msg;
	cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb).toImageMsg(msg.rgb);
	cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth).toImageMsg(msg.depth);

	cv_bridge::CvImagePtr rgbPtr, depthPtr;
	toCvCopy(msg, rgbPtr, depthPtr);

	ASSERT_TRUE(rgbPtr && depthPtr);
	EXPECT_EQ(cv::countNonZero(rgbPtr->image.reshape(1) != rgb.reshape(1)), 0);
	EXPECT_EQ(cv::countNonZero(depthPtr->image != depth), 0);

	// The copy must be independent of the message buffer.
	rgbPtr->image.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0);
	EXPECT_EQ(rgb.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));
}

TEST(MsgConversion, toCvCopyEmptyImageYieldsEmptyPtr)
{
	rtabmap_msgs::msg::RGBDImage msg;

	cv_bridge::CvImagePtr rgbPtr, depthPtr;
	toCvCopy(msg, rgbPtr, depthPtr);

	ASSERT_TRUE(rgbPtr && depthPtr) << "pointers must be valid even with no image";
	EXPECT_TRUE(rgbPtr->image.empty());
	EXPECT_TRUE(depthPtr->image.empty());
}

TEST(MsgConversion, toCvShareAliasesRawImages)
{
	cv::Mat rgb(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
	cv::Mat depth(4, 4, CV_16UC1, cv::Scalar(1000));

	rtabmap_msgs::msg::RGBDImage msg;
	cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb).toImageMsg(msg.rgb);
	cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth).toImageMsg(msg.depth);

	cv_bridge::CvImageConstPtr rgbPtr, depthPtr;
	toCvShare(msg, std::shared_ptr<void const>(), rgbPtr, depthPtr);

	ASSERT_TRUE(rgbPtr && depthPtr);
	ASSERT_FALSE(rgbPtr->image.empty());
	EXPECT_EQ(cv::countNonZero(rgbPtr->image.reshape(1) != rgb.reshape(1)), 0);
	EXPECT_EQ(cv::countNonZero(depthPtr->image != depth), 0);
}

/////////////////////////
// Compressed images
/////////////////////////

namespace {

/// Builds a depth image compressed the way rtabmap does it (not a jpg/png CompressedImage).
sensor_msgs::msg::CompressedImage makeRtabmapCompressedDepth(const cv::Mat & depth)
{
	sensor_msgs::msg::CompressedImage msg;
	msg.format = "";   // anything but "jpg" takes the rtabmap::uncompressImage path
	msg.data = rtabmap::compressImage(depth, ".png");
	return msg;
}

}  // namespace

TEST(MsgConversion, toCvCopyReadsCompressedDepth)
{
	const cv::Mat depth(4, 4, CV_16UC1, cv::Scalar(1234));

	rtabmap_msgs::msg::RGBDImage msg;
	msg.depth_compressed = makeRtabmapCompressedDepth(depth);

	cv_bridge::CvImagePtr rgbPtr, depthPtr;
	toCvCopy(msg, rgbPtr, depthPtr);

	ASSERT_TRUE(depthPtr);
	ASSERT_FALSE(depthPtr->image.empty());
	EXPECT_EQ(depthPtr->image.type(), CV_16UC1);
	EXPECT_EQ(depthPtr->encoding, sensor_msgs::image_encodings::TYPE_16UC1);
	EXPECT_EQ(cv::countNonZero(depthPtr->image != depth), 0);
}

TEST(MsgConversion, toCvShareReadsCompressedDepth)
{
	const cv::Mat depth(4, 4, CV_32FC1, cv::Scalar(1.5f));

	rtabmap_msgs::msg::RGBDImage msg;
	msg.depth_compressed = makeRtabmapCompressedDepth(depth);

	cv_bridge::CvImageConstPtr rgbPtr, depthPtr;
	toCvShare(msg, std::shared_ptr<void const>(), rgbPtr, depthPtr);

	ASSERT_TRUE(depthPtr);
	ASSERT_FALSE(depthPtr->image.empty());
	EXPECT_EQ(depthPtr->image.type(), CV_32FC1);
	EXPECT_EQ(depthPtr->encoding, sensor_msgs::image_encodings::TYPE_32FC1);
	EXPECT_EQ(cv::countNonZero(depthPtr->image != depth), 0);
}

TEST(MsgConversion, toCvCopyReadsCompressedRgb)
{
	const cv::Mat rgb(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));

	rtabmap_msgs::msg::RGBDImage msg;
	msg.rgb_compressed.format = "png";
	msg.rgb_compressed.data = rtabmap::compressImage(rgb, ".png");

	cv_bridge::CvImagePtr rgbPtr, depthPtr;
	toCvCopy(msg, rgbPtr, depthPtr);

	ASSERT_TRUE(rgbPtr);
	ASSERT_FALSE(rgbPtr->image.empty());
	EXPECT_EQ(rgbPtr->image.type(), CV_8UC3);
	EXPECT_EQ(cv::countNonZero(rgbPtr->image.reshape(1) != rgb.reshape(1)), 0);
}

/////////////////////////
// SensorData: raw copies, laser scans, stereo
/////////////////////////

TEST(MsgConversion, sensorDataToROSCopyRawDataCarriesImages)
{
	cv::Mat K = (cv::Mat_<double>(3, 3) <<
			525.0, 0.0, 4.0, 0.0, 525.0, 4.0, 0.0, 0.0, 1.0);
	const rtabmap::CameraModel model("cam", cv::Size(8, 8), K, cv::Mat(), cv::Mat(), cv::Mat());

	const cv::Mat rgb(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
	const cv::Mat depth(8, 8, CV_16UC1, cv::Scalar(2000));
	rtabmap::SensorData in(rgb, depth, model, 1, 1000.0);

	// Without copyRawData the raw images are not serialized...
	rtabmap_msgs::msg::SensorData without;
	sensorDataToROS(in, without, "base_link", /*copyRawData=*/false);
	EXPECT_TRUE(without.left.data.empty());
	EXPECT_TRUE(without.right.data.empty());

	// ...with it, they are.
	rtabmap_msgs::msg::SensorData with;
	sensorDataToROS(in, with, "base_link", /*copyRawData=*/true);
	ASSERT_FALSE(with.left.data.empty());
	ASSERT_FALSE(with.right.data.empty());
	EXPECT_EQ(with.left.encoding, sensor_msgs::image_encodings::BGR8);
	EXPECT_EQ(with.right.encoding, sensor_msgs::image_encodings::TYPE_16UC1);

	const rtabmap::SensorData out = sensorDataFromROS(with);
	ASSERT_FALSE(out.imageRaw().empty());
	EXPECT_EQ(cv::countNonZero(out.imageRaw().reshape(1) != rgb.reshape(1)), 0);
	ASSERT_FALSE(out.depthRaw().empty());
	EXPECT_EQ(cv::countNonZero(out.depthRaw() != depth), 0);
}

TEST(MsgConversion, sensorDataLaserScanRoundTrip)
{
	cv::Mat points(1, 3, CV_32FC3);
	points.at<cv::Vec3f>(0, 0) = cv::Vec3f(1.0f, 0.0f, 0.0f);
	points.at<cv::Vec3f>(0, 1) = cv::Vec3f(0.0f, 2.0f, 0.0f);
	points.at<cv::Vec3f>(0, 2) = cv::Vec3f(0.0f, 0.0f, 3.0f);

	const rtabmap::Transform localTransform(0.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);
	const rtabmap::LaserScan scan(points, /*maxPoints=*/100, /*maxRange=*/40.0f,
			rtabmap::LaserScan::kXYZ, localTransform);

	rtabmap::SensorData in;
	in.setStamp(1000.0);
	in.setLaserScan(scan);

	rtabmap_msgs::msg::SensorData msg;
	sensorDataToROS(in, msg, "base_link", /*copyRawData=*/true);

	EXPECT_EQ(msg.laser_scan_max_pts, 100);
	EXPECT_FLOAT_EQ(msg.laser_scan_max_range, 40.0f);
	EXPECT_EQ(msg.laser_scan_format, (int)rtabmap::LaserScan::kXYZ);
	expectTransformNear(transformFromGeometryMsg(msg.laser_scan_local_transform),
			localTransform, 1e-4f);

	const rtabmap::SensorData out = sensorDataFromROS(msg);
	const rtabmap::LaserScan & outScan = out.laserScanRaw().empty()
			? out.laserScanCompressed() : out.laserScanRaw();
	EXPECT_EQ(outScan.size(), scan.size());
	EXPECT_EQ(outScan.maxPoints(), scan.maxPoints());
	EXPECT_FLOAT_EQ(outScan.rangeMax(), scan.rangeMax());
	expectTransformNear(outScan.localTransform(), localTransform, 1e-4f);
}

TEST(MsgConversion, sensorDataStereoModelRoundTrip)
{
	const double fx = 525.0;
	const double baseline = 0.12;
	const rtabmap::Transform localTransform(0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f);

	const rtabmap::StereoCameraModel stereo(
			fx, fx, 320.0, 240.0, baseline, localTransform, cv::Size(640, 480));
	ASSERT_TRUE(stereo.isValidForProjection()) << "precondition";

	rtabmap::SensorData in;
	in.setStamp(1000.0);
	in.setStereoImage(cv::Mat(), cv::Mat(), stereo);

	rtabmap_msgs::msg::SensorData msg;
	sensorDataToROS(in, msg, "base_link");

	// The stereo branch fills BOTH camera infos, unlike the monocular one.
	ASSERT_EQ(msg.left_camera_info.size(), 1u);
	ASSERT_EQ(msg.right_camera_info.size(), 1u);

	const rtabmap::SensorData out = sensorDataFromROS(msg);
	ASSERT_EQ(out.stereoCameraModels().size(), 1u);
	EXPECT_TRUE(out.cameraModels().empty()) << "must not be read back as monocular";
	EXPECT_NEAR(out.stereoCameraModels()[0].left().fx(), fx, 1e-9);
	EXPECT_NEAR(out.stereoCameraModels()[0].baseline(), baseline, 1e-6);
	expectTransformNear(out.stereoCameraModels()[0].localTransform(), localTransform, 1e-4f);
}

TEST(MsgConversion, nodeWithStereoModelRoundTrip)
{
	const rtabmap::StereoCameraModel stereo(
			525.0, 525.0, 320.0, 240.0, 0.12,
			rtabmap::Transform::getIdentity(), cv::Size(640, 480));

	rtabmap::Signature in(3, 0, 1, 1000.0, "stereo_node", sampleTransform());
	in.sensorData().setStereoImage(cv::Mat(), cv::Mat(), stereo);

	rtabmap_msgs::msg::Node msg;
	nodeToROS(in, msg);
	const rtabmap::Signature out = nodeFromROS(msg);

	EXPECT_EQ(out.id(), in.id());
	ASSERT_EQ(out.sensorData().stereoCameraModels().size(), 1u);
	EXPECT_NEAR(out.sensorData().stereoCameraModels()[0].baseline(), 0.12, 1e-6);
}

TEST(MsgConversion, infoOdomCacheRoundTrip)
{
	// Statistics carries a whole MapGraph for the odometry cache in localization mode.
	std::map<int, rtabmap::Transform> poses;
	poses.insert(std::make_pair(1, sampleTransform()));
	poses.insert(std::make_pair(2, rtabmap::Transform(1, 2, 3, 0, 0, 0)));

	std::multimap<int, rtabmap::Link> links;
	links.insert(std::make_pair(1, rtabmap::Link(
			1, 2, rtabmap::Link::kNeighbor, sampleTransform())));

	rtabmap::Statistics in;
	in.setExtended(true);
	in.setOdomCachePoses(poses);
	in.setOdomCacheConstraints(links);

	rtabmap_msgs::msg::Info msg;
	infoToROS(in, msg);
	ASSERT_EQ(msg.odom_cache.poses.size(), poses.size());
	ASSERT_EQ(msg.odom_cache.links.size(), links.size());

	rtabmap::Statistics out;
	infoFromROS(msg, out);

	ASSERT_EQ(out.odomCachePoses().size(), poses.size());
	expectTransformNear(out.odomCachePoses().at(1), poses.at(1));
	expectTransformNear(out.odomCachePoses().at(2), poses.at(2));
	EXPECT_EQ(out.odomCacheConstraints().size(), links.size());
}

/////////////////////////
// TF-based conversions
/////////////////////////

namespace {

/// A tf2 buffer needs a clock, but neither a node nor a listener: transforms can be
/// injected directly, which makes every TF-based conversion an ordinary unit test.
std::shared_ptr<tf2_ros::Buffer> makeTfBuffer()
{
	std::shared_ptr<tf2_ros::Buffer> buffer =
			std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
	// Transforms are injected synchronously before the lookups, so tell tf2 not to warn
	// about waiting for a listener thread that will never exist.
	buffer->setUsingDedicatedThread(true);
	return buffer;
}

void addTf(tf2_ros::Buffer & buffer,
		const std::string & parent, const std::string & child,
		const rtabmap::Transform & t, double stamp, bool isStatic = true)
{
	geometry_msgs::msg::TransformStamped msg;
	msg.header.stamp = timestampToROS(stamp);
	msg.header.frame_id = parent;
	msg.child_frame_id = child;
	transformToGeometryMsg(t, msg.transform);
	ASSERT_TRUE(buffer.setTransform(msg, "unit_test", isStatic));
}

}  // namespace

TEST(MsgConversion, getTransformReadsTheBuffer)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	const rtabmap::Transform baseToCamera(0.1f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f);
	addTf(*buffer, "base_link", "camera_link", baseToCamera, 1000.0);

	const rtabmap::Transform out =
			getTransform("base_link", "camera_link", timestampToROS(1000.0), *buffer, 0.0);

	expectTransformNear(out, baseToCamera);
}

TEST(MsgConversion, getTransformReturnsNullWhenUnknown)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "camera_link", rtabmap::Transform::getIdentity(), 1000.0);

	// An unrelated frame must not throw; it must come back as a null transform.
	EXPECT_TRUE(getTransform("base_link", "lidar_link", timestampToROS(1000.0), *buffer, 0.0)
			.isNull());
}

TEST(MsgConversion, getTransformIsInvertedByFrameOrder)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	const rtabmap::Transform baseToCamera(0.1f, 0.2f, 0.3f, 0.0f, 0.0f, 0.5f);
	addTf(*buffer, "base_link", "camera_link", baseToCamera, 1000.0);

	const rtabmap::Transform forward =
			getTransform("base_link", "camera_link", timestampToROS(1000.0), *buffer, 0.0);
	const rtabmap::Transform backward =
			getTransform("camera_link", "base_link", timestampToROS(1000.0), *buffer, 0.0);

	expectTransformNear(backward, forward.inverse(), 1e-4f);
}

TEST(MsgConversion, getMovingTransformMeasuresMotionBetweenStamps)
{
	// base_link drives 1 m along x of odom between t=1000 and t=1001.
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "odom", "base_link", rtabmap::Transform(0, 0, 0, 0, 0, 0), 1000.0, false);
	addTf(*buffer, "odom", "base_link", rtabmap::Transform(1, 0, 0, 0, 0, 0), 1001.0, false);

	// Motion of base_link from t=1000 to t=1001, seen in the fixed odom frame.
	const rtabmap::Transform motion = getMovingTransform(
			"base_link", "odom", timestampToROS(1000.0), timestampToROS(1001.0), *buffer, 0.0);

	ASSERT_FALSE(motion.isNull());
	EXPECT_NEAR(motion.x(), 1.0, 1e-4);
	EXPECT_NEAR(motion.y(), 0.0, 1e-4);
	EXPECT_NEAR(motion.z(), 0.0, 1e-4);
}

TEST(MsgConversion, getMovingTransformInterpolatesBetweenStamps)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "odom", "base_link", rtabmap::Transform(0, 0, 0, 0, 0, 0), 1000.0, false);
	addTf(*buffer, "odom", "base_link", rtabmap::Transform(1, 0, 0, 0, 0, 0), 1001.0, false);

	// Halfway through, so half the motion.
	const rtabmap::Transform half = getMovingTransform(
			"base_link", "odom", timestampToROS(1000.0), timestampToROS(1000.5), *buffer, 0.0);

	ASSERT_FALSE(half.isNull());
	EXPECT_NEAR(half.x(), 0.5, 1e-4);
}

TEST(MsgConversion, getMovingTransformIsNullWithoutAFixedFrame)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "odom", "base_link", rtabmap::Transform::getIdentity(), 1000.0, false);

	EXPECT_TRUE(getMovingTransform("base_link", "map",
			timestampToROS(1000.0), timestampToROS(1001.0), *buffer, 0.0).isNull());
}

TEST(MsgConversion, convertScanMsgProducesALaserScan)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	const rtabmap::Transform baseToLaser(0.2f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f);
	addTf(*buffer, "base_link", "laser", baseToLaser, 1000.0);

	sensor_msgs::msg::LaserScan msg;
	msg.header.stamp = timestampToROS(1000.0);
	msg.header.frame_id = "laser";
	msg.angle_min = -1.0f;
	msg.angle_max = 1.0f;
	msg.angle_increment = 0.1f;
	msg.time_increment = 0.0f;
	msg.range_min = 0.1f;
	msg.range_max = 30.0f;
	msg.ranges.assign(21, 5.0f);

	rtabmap::LaserScan scan;
	ASSERT_TRUE(convertScanMsg(msg, "base_link", "", timestampToROS(1000.0),
			scan, *buffer, 0.0));

	EXPECT_FALSE(scan.empty());
	EXPECT_EQ(scan.size(), (int)msg.ranges.size());
	expectTransformNear(scan.localTransform(), baseToLaser, 1e-4f);
}

TEST(MsgConversion, convertScanMsgRejectsMalformedScans)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "laser", rtabmap::Transform::getIdentity(), 1000.0);

	sensor_msgs::msg::LaserScan base;
	base.header.stamp = timestampToROS(1000.0);
	base.header.frame_id = "laser";
	base.angle_min = -1.0f;
	base.angle_max = 1.0f;
	base.angle_increment = 0.1f;
	base.range_min = 0.1f;
	base.range_max = 30.0f;
	base.ranges.assign(21, 5.0f);

	rtabmap::LaserScan scan;

	sensor_msgs::msg::LaserScan zeroIncrement = base;
	zeroIncrement.angle_increment = 0.0f;
	EXPECT_FALSE(convertScanMsg(zeroIncrement, "base_link", "", timestampToROS(1000.0),
			scan, *buffer, 0.0)) << "angle_increment of 0 would divide by zero";

	sensor_msgs::msg::LaserScan invertedRange = base;
	invertedRange.range_min = 40.0f;
	EXPECT_FALSE(convertScanMsg(invertedRange, "base_link", "", timestampToROS(1000.0),
			scan, *buffer, 0.0)) << "range_min > range_max";

	sensor_msgs::msg::LaserScan invertedAngle = base;
	invertedAngle.angle_min = 1.0f;
	invertedAngle.angle_max = -1.0f;
	EXPECT_FALSE(convertScanMsg(invertedAngle, "base_link", "", timestampToROS(1000.0),
			scan, *buffer, 0.0)) << "positive increment with angle_max < angle_min";
}

TEST(MsgConversion, convertScanMsgFailsWithoutTf)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();   // empty

	sensor_msgs::msg::LaserScan msg;
	msg.header.stamp = timestampToROS(1000.0);
	msg.header.frame_id = "laser";
	msg.angle_min = -1.0f;
	msg.angle_max = 1.0f;
	msg.angle_increment = 0.1f;
	msg.range_min = 0.1f;
	msg.range_max = 30.0f;
	msg.ranges.assign(21, 5.0f);

	rtabmap::LaserScan scan;
	EXPECT_FALSE(convertScanMsg(msg, "base_link", "", timestampToROS(1000.0),
			scan, *buffer, 0.0));
}

TEST(MsgConversion, convertScan3dMsgKeepsLocalTransformAndLimits)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	const rtabmap::Transform baseToLidar(0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f);
	addTf(*buffer, "base_link", "lidar", baseToLidar, 1000.0);

	sensor_msgs::msg::PointCloud2 msg =
			makeXYZCloud({{1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}});
	msg.header.stamp = timestampToROS(1000.0);
	msg.header.frame_id = "lidar";

	rtabmap::LaserScan scan;
	ASSERT_TRUE(convertScan3dMsg(msg, "base_link", "", timestampToROS(1000.0),
			scan, *buffer, 0.0));

	EXPECT_EQ(scan.size(), 3);
	expectTransformNear(scan.localTransform(), baseToLidar, 1e-4f);
	EXPECT_EQ(scan.rangeMax(), 0.0f) << "no max range requested";

	rtabmap::LaserScan limited;
	ASSERT_TRUE(convertScan3dMsg(msg, "base_link", "", timestampToROS(1000.0),
			limited, *buffer, 0.0, /*maxPoints=*/10, /*maxRange=*/2.5f));
	EXPECT_EQ(limited.maxPoints(), 10);
	EXPECT_FLOAT_EQ(limited.rangeMax(), 2.5f);
}

TEST(MsgConversion, convertScan3dMsgFailsWithoutTf)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();   // empty

	sensor_msgs::msg::PointCloud2 msg = makeXYZCloud({{1.0f, 0.0f, 0.0f}});
	msg.header.stamp = timestampToROS(1000.0);
	msg.header.frame_id = "lidar";

	rtabmap::LaserScan scan;
	EXPECT_FALSE(convertScan3dMsg(msg, "base_link", "", timestampToROS(1000.0),
			scan, *buffer, 0.0));
}

TEST(MsgConversion, landmarksFromROSAppliesTfAndDefaultVariance)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	const rtabmap::Transform baseToCamera(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	addTf(*buffer, "base_link", "camera_link", baseToCamera, 1000.0);

	geometry_msgs::msg::PoseWithCovarianceStamped tag;
	tag.header.stamp = timestampToROS(1000.0);
	tag.header.frame_id = "camera_link";
	tag.pose.pose.position.x = 2.0;      // 2 m in front of the camera
	tag.pose.pose.orientation.w = 1.0;
	// covariance left at zero -> the defaults must be substituted

	std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> > tags;
	tags.insert(std::make_pair(7, std::make_pair(tag, 0.15f)));

	const rtabmap::Landmarks landmarks = landmarksFromROS(
			tags, "base_link", "", timestampToROS(1000.0), *buffer, 0.0,
			/*defaultLinVariance=*/0.01, /*defaultAngVariance=*/0.02);

	ASSERT_EQ(landmarks.size(), 1u);
	ASSERT_TRUE(landmarks.find(7) != landmarks.end());

	// The tag pose must come back in base_link: 0.5 (base->camera) + 2.0 (camera->tag).
	EXPECT_NEAR(landmarks.at(7).pose().x(), 2.5, 1e-4);

	const cv::Mat cov = landmarks.at(7).covariance();
	ASSERT_EQ(cov.rows, 6);
	EXPECT_NEAR(cov.at<double>(0,0), 0.01, 1e-9) << "linear default";
	EXPECT_NEAR(cov.at<double>(3,3), 0.02, 1e-9) << "angular default";
}

TEST(MsgConversion, landmarksFromROSCorrectsForOdometryMotion)
{
	// The tag is seen 1 s after the odometry stamp, during which the robot drives 1 m.
	// landmarksFromROS must fold that motion in, otherwise the landmark is placed where
	// the robot would have seen it had it not moved.
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	const rtabmap::Transform baseToCamera(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	addTf(*buffer, "base_link", "camera_link", baseToCamera, 1000.0);
	addTf(*buffer, "odom", "base_link", rtabmap::Transform(0, 0, 0, 0, 0, 0), 1000.0, false);
	addTf(*buffer, "odom", "base_link", rtabmap::Transform(1, 0, 0, 0, 0, 0), 1001.0, false);

	geometry_msgs::msg::PoseWithCovarianceStamped tag;
	tag.header.stamp = timestampToROS(1001.0);     // observed at t=1001
	tag.header.frame_id = "camera_link";
	tag.pose.pose.position.x = 2.0;
	tag.pose.pose.orientation.w = 1.0;

	std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> > tags;
	tags.insert(std::make_pair(7, std::make_pair(tag, 0.15f)));

	// odomStamp is 1000, one second BEFORE the observation.
	const rtabmap::Landmarks corrected = landmarksFromROS(
			tags, "base_link", "odom", timestampToROS(1000.0), *buffer, 0.0, 0.01, 0.02);

	ASSERT_EQ(corrected.size(), 1u);
	// 0.5 (base->camera) + 2.0 (camera->tag) + 1.0 (odometry motion since odomStamp).
	EXPECT_NEAR(corrected.at(7).pose().x(), 3.5, 1e-3);

	// Without an odom frame the correction cannot be looked up, and the landmark stays
	// in the frame at the observation stamp.
	const rtabmap::Landmarks uncorrected = landmarksFromROS(
			tags, "base_link", "", timestampToROS(1000.0), *buffer, 0.0, 0.01, 0.02);
	ASSERT_EQ(uncorrected.size(), 1u);
	EXPECT_NEAR(uncorrected.at(7).pose().x(), 2.5, 1e-3);
}

TEST(MsgConversion, landmarksFromROSKeepsProvidedCovariance)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "camera_link", rtabmap::Transform::getIdentity(), 1000.0);

	geometry_msgs::msg::PoseWithCovarianceStamped tag;
	tag.header.stamp = timestampToROS(1000.0);
	tag.header.frame_id = "camera_link";
	tag.pose.pose.position.x = 1.0;
	tag.pose.pose.orientation.w = 1.0;
	for(size_t i=0; i<6; ++i)
	{
		tag.pose.covariance[i*6 + i] = 0.5;   // a real, finite covariance
	}

	std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> > tags;
	tags.insert(std::make_pair(1, std::make_pair(tag, 0.1f)));

	const rtabmap::Landmarks landmarks = landmarksFromROS(
			tags, "base_link", "", timestampToROS(1000.0), *buffer, 0.0,
			/*defaultLinVariance=*/0.01, /*defaultAngVariance=*/0.02);

	ASSERT_EQ(landmarks.size(), 1u);
	EXPECT_NEAR(landmarks.at(1).covariance().at<double>(0,0), 0.5, 1e-9)
		<< "a provided covariance must not be replaced by the default";
	EXPECT_NEAR(landmarks.at(1).covariance().at<double>(3,3), 0.5, 1e-9);
}

TEST(MsgConversion, landmarksFromROSRejectsNonPositiveIds)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "camera_link", rtabmap::Transform::getIdentity(), 1000.0);

	geometry_msgs::msg::PoseWithCovarianceStamped tag;
	tag.header.stamp = timestampToROS(1000.0);
	tag.header.frame_id = "camera_link";
	tag.pose.pose.orientation.w = 1.0;

	std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> > tags;
	tags.insert(std::make_pair(0, std::make_pair(tag, 0.1f)));
	tags.insert(std::make_pair(-3, std::make_pair(tag, 0.1f)));
	tags.insert(std::make_pair(5, std::make_pair(tag, 0.1f)));

	const rtabmap::Landmarks landmarks = landmarksFromROS(
			tags, "base_link", "", timestampToROS(1000.0), *buffer, 0.0, 0.01, 0.02);

	EXPECT_EQ(landmarks.size(), 1u) << "ids <= 0 must be dropped";
	EXPECT_TRUE(landmarks.find(5) != landmarks.end());
}

void expectTfDeskewRecoversWall(bool slerp)
{
	SCOPED_TRACE(slerp ? "slerp=true" : "slerp=false");

	// base_link advances 0.1 m along odom over the sweep -- the same motion the constant
	// velocity tests apply at 1 m/s. With slerp the correction is interpolated between
	// the two end poses; without it, every sample gets its own TF lookup.
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "odom", "base_link", rtabmap::Transform(0, 0, 0, 0, 0, 0), 1000.0, false);
	addTf(*buffer, "odom", "base_link",
			rtabmap::Transform(float(kSpeed * kScanSpan), 0, 0, 0, 0, 0),
			1000.0 + kScanSpan, false);

	sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(1000.0, 0.0);
	in.header.frame_id = "base_link";

	sensor_msgs::msg::PointCloud2 out;
	ASSERT_TRUE(deskew(in, out, "odom", *buffer, 0.0, slerp));

	for(size_t i=0; i<kScanPoints; ++i)
	{
		EXPECT_NEAR(readWallX(out, i, 0, kTimeOnColumns), kWallDistance, 1e-3)
			<< "sample " << i;
	}
}

TEST(MsgConversion, deskewWithTfBufferSlerp)
{
	expectTfDeskewRecoversWall(/*slerp=*/true);
}

TEST(MsgConversion, deskewWithTfBufferPerPointLookup)
{
	// slerp=false takes a completely different path: a getMovingTransform() per sample
	// instead of one interpolation between the sweep's end poses.
	expectTfDeskewRecoversWall(/*slerp=*/false);
}

TEST(MsgConversion, deskewSlerpLinearizesNonLinearMotion)
{
	// A piecewise-linear trajectory: the robot covers most of the sweep's distance in the
	// first half, then nearly stops. Both the TF buffer and the skew of the input cloud
	// are generated from this same motion, so the true answer is unambiguous: a correct
	// deskew must recover the flat wall.
	const double kneeTime = kScanSpan / 2.0;
	const double kneeX = 0.09;                        // vs 0.0495 if it were linear
	const double endX = kSpeed * kScanSpan;           // 0.099
	// Matches how tf2 interpolates between consecutive samples.
	auto travelled = [&](double elapsed) {
		return elapsed <= kneeTime
				? kneeX * (elapsed / kneeTime)
				: kneeX + (endX - kneeX) * ((elapsed - kneeTime) / (kScanSpan - kneeTime));
	};

	auto buildBuffer = [&]() {
		std::shared_ptr<tf2_ros::Buffer> b = makeTfBuffer();
		geometry_msgs::msg::TransformStamped m;
		m.header.frame_id = "odom";
		m.child_frame_id = "base_link";
		m.transform.rotation.w = 1.0;
		for(double elapsed : {0.0, kneeTime, kScanSpan})
		{
			m.header.stamp = timestampToROS(1000.0 + elapsed);
			m.transform.translation.x = travelled(elapsed);
			b->setTransform(m, "unit_test", false);
		}
		return b;
	};

	sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(
			1000.0, 0.0, kOffsetSecFloat32, kTimeOnColumns, 1, "t", false, travelled);
	in.header.frame_id = "base_link";

	sensor_msgs::msg::PointCloud2 slerped, perPoint;
	const std::shared_ptr<tf2_ros::Buffer> b1 = buildBuffer();
	const std::shared_ptr<tf2_ros::Buffer> b2 = buildBuffer();
	ASSERT_TRUE(deskew(in, slerped, "odom", *b1, 0.0, /*slerp=*/true));
	ASSERT_TRUE(deskew(in, perPoint, "odom", *b2, 0.0, /*slerp=*/false));

	// Per-point lookups follow the real motion, so they reconstruct the wall exactly.
	for(size_t i=0; i<kScanPoints; ++i)
	{
		EXPECT_NEAR(readWallX(perPoint, i, 0, kTimeOnColumns), kWallDistance, 1e-3)
			<< "slerp=false must be exact, sample " << i;
	}

	// slerp only reads the sweep's two end poses, so it straight-lines through the knee
	// and leaves a visible residual in the middle of the scan.
	double worst = 0.0;
	for(size_t i=0; i<kScanPoints; ++i)
	{
		worst = std::max(worst,
				std::abs(double(readWallX(slerped, i, 0, kTimeOnColumns)) - kWallDistance));
	}
	EXPECT_GT(worst, 1e-2) << "slerp must show the error of linearizing the motion";
}

TEST(MsgConversion, deskewWithTfBufferFailsWithoutTf)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();   // empty

	sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(1000.0, 0.0);
	in.header.frame_id = "base_link";

	sensor_msgs::msg::PointCloud2 out;
	EXPECT_FALSE(deskew(in, out, "odom", *buffer, 0.0, true));
}

/////////////////////////
// convertRGBDMsgs / convertStereoMsg
/////////////////////////

namespace {

/// A rectified pinhole CameraInfo. tx is P(0,3): 0 for the left/depth camera, and
/// -fx*baseline for the right camera of a stereo pair.
sensor_msgs::msg::CameraInfo makeCameraInfo(
		const std::string & frameId, double stamp, int width, int height,
		double tx = 0.0, double fx = 100.0)
{
	sensor_msgs::msg::CameraInfo info;
	info.header.stamp = timestampToROS(stamp);
	info.header.frame_id = frameId;
	info.width = width;
	info.height = height;
	info.distortion_model = "plumb_bob";
	info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
	info.k = {fx, 0.0, width/2.0, 0.0, fx, height/2.0, 0.0, 0.0, 1.0};
	info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	info.p = {fx, 0.0, width/2.0, tx, 0.0, fx, height/2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
	return info;
}

cv_bridge::CvImageConstPtr makeImage(
		const std::string & frameId, double stamp,
		const cv::Mat & image, const std::string & encoding)
{
	std_msgs::msg::Header header;
	header.stamp = timestampToROS(stamp);
	header.frame_id = frameId;
	return std::make_shared<cv_bridge::CvImage>(header, encoding, image);
}

}  // namespace

TEST(MsgConversion, convertRGBDMsgsSingleCamera)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	const rtabmap::Transform baseToCamera(0.1f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f);
	addTf(*buffer, "base_link", "camera_link", baseToCamera, 1000.0);

	const cv::Mat rgbImage(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
	const cv::Mat depthImage(8, 8, CV_16UC1, cv::Scalar(1500));

	const std::vector<cv_bridge::CvImageConstPtr> images =
			{makeImage("camera_link", 1000.0, rgbImage, "bgr8")};
	const std::vector<cv_bridge::CvImageConstPtr> depths =
			{makeImage("camera_link", 1000.0, depthImage, "16UC1")};
	const std::vector<sensor_msgs::msg::CameraInfo> infos =
			{makeCameraInfo("camera_link", 1000.0, 8, 8)};

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	ASSERT_TRUE(convertRGBDMsgs(images, depths, infos, {}, "base_link", "",
			timestampToROS(1000.0), rgb, depth, models, stereoModels,
			*buffer, 0.0, /*alreadyRectifiedImages=*/true));

	EXPECT_TRUE(stereoModels.empty()) << "a depth image must not produce a stereo model";
	ASSERT_EQ(models.size(), 1u);
	EXPECT_NEAR(models[0].fx(), 100.0, 1e-9);
	expectTransformNear(models[0].localTransform(), baseToCamera, 1e-4f);

	ASSERT_EQ(rgb.cols, 8);
	ASSERT_EQ(rgb.rows, 8);
	EXPECT_EQ(depth.type(), CV_16UC1);
	EXPECT_EQ(depth.at<unsigned short>(0, 0), 1500);
}

TEST(MsgConversion, convertRGBDMsgsMultiCameraSideBySide)
{
	// Two cameras are concatenated horizontally into one wide image, one model each.
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "cam0", rtabmap::Transform(0.1f, 0.1f, 0, 0, 0, 0), 1000.0);
	addTf(*buffer, "base_link", "cam1", rtabmap::Transform(0.1f, -0.1f, 0, 0, 0, 0), 1000.0);

	const cv::Mat rgb0(8, 8, CV_8UC3, cv::Scalar(10, 0, 0));
	const cv::Mat rgb1(8, 8, CV_8UC3, cv::Scalar(0, 20, 0));
	const cv::Mat depth0(8, 8, CV_16UC1, cv::Scalar(1000));
	const cv::Mat depth1(8, 8, CV_16UC1, cv::Scalar(2000));

	const std::vector<cv_bridge::CvImageConstPtr> images = {
			makeImage("cam0", 1000.0, rgb0, "bgr8"),
			makeImage("cam1", 1000.0, rgb1, "bgr8")};
	const std::vector<cv_bridge::CvImageConstPtr> depths = {
			makeImage("cam0", 1000.0, depth0, "16UC1"),
			makeImage("cam1", 1000.0, depth1, "16UC1")};
	const std::vector<sensor_msgs::msg::CameraInfo> infos = {
			makeCameraInfo("cam0", 1000.0, 8, 8),
			makeCameraInfo("cam1", 1000.0, 8, 8)};

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	ASSERT_TRUE(convertRGBDMsgs(images, depths, infos, {}, "base_link", "",
			timestampToROS(1000.0), rgb, depth, models, stereoModels,
			*buffer, 0.0, true));

	ASSERT_EQ(models.size(), 2u);
	EXPECT_EQ(rgb.cols, 16) << "the two 8-wide images must be side by side";
	EXPECT_EQ(rgb.rows, 8);
	EXPECT_EQ(depth.cols, 16);

	// Each half keeps its own camera's data.
	EXPECT_EQ(depth.at<unsigned short>(0, 0), 1000);
	EXPECT_EQ(depth.at<unsigned short>(0, 8), 2000);
	EXPECT_NEAR(models[0].localTransform().y(), 0.1, 1e-4);
	EXPECT_NEAR(models[1].localTransform().y(), -0.1, 1e-4);
}

namespace {

/// base_link sits at odom origin at t=1000 and 1 m along x at t=1001.
void addOdomMotion(tf2_ros::Buffer & buffer)
{
	geometry_msgs::msg::TransformStamped m;
	m.header.frame_id = "odom";
	m.child_frame_id = "base_link";
	m.transform.rotation.w = 1.0;
	m.header.stamp = timestampToROS(1000.0);
	m.transform.translation.x = 0.0;
	ASSERT_TRUE(buffer.setTransform(m, "unit_test", false));
	m.header.stamp = timestampToROS(1001.0);
	m.transform.translation.x = 1.0;
	ASSERT_TRUE(buffer.setTransform(m, "unit_test", false));
}

}  // namespace

TEST(MsgConversion, convertRGBDMsgsSyncsToOdomStamp)
{
	// The image is captured at t=1001 but must be expressed relative to the base frame
	// at odomStamp=1000, one metre back.
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	const rtabmap::Transform baseToCamera(0.1f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f);
	addTf(*buffer, "base_link", "camera_link", baseToCamera, 1000.0);
	addOdomMotion(*buffer);

	const cv::Mat rgbImage(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
	const cv::Mat depthImage(8, 8, CV_16UC1, cv::Scalar(1500));
	const std::vector<cv_bridge::CvImageConstPtr> images =
			{makeImage("camera_link", 1001.0, rgbImage, "bgr8")};
	const std::vector<cv_bridge::CvImageConstPtr> depths =
			{makeImage("camera_link", 1001.0, depthImage, "16UC1")};
	const std::vector<sensor_msgs::msg::CameraInfo> infos =
			{makeCameraInfo("camera_link", 1001.0, 8, 8)};

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> corrected;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	ASSERT_TRUE(convertRGBDMsgs(images, depths, infos, {}, "base_link", "odom",
			timestampToROS(1000.0), rgb, depth, corrected, stereoModels, *buffer, 0.0, true));
	ASSERT_EQ(corrected.size(), 1u);
	EXPECT_NEAR(corrected[0].localTransform().x(), 1.1, 1e-3)
		<< "0.1 base->camera plus 1.0 of odometry motion";

	// Without an odom frame the motion is not folded in.
	std::vector<rtabmap::CameraModel> uncorrected;
	std::vector<rtabmap::StereoCameraModel> stereoModels2;
	ASSERT_TRUE(convertRGBDMsgs(images, depths, infos, {}, "base_link", "",
			timestampToROS(1000.0), rgb, depth, uncorrected, stereoModels2, *buffer, 0.0, true));
	ASSERT_EQ(uncorrected.size(), 1u);
	EXPECT_NEAR(uncorrected[0].localTransform().x(), 0.1, 1e-3);
}

TEST(MsgConversion, convertRGBDMsgsSyncsEachCameraAtItsOwnStamp)
{
	// Two cameras captured 1 s apart, on a robot moving 1 m/s along x. Each must be
	// corrected by its OWN elapsed motion, not by a single shared one.
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "cam0", rtabmap::Transform(0.1f, 0.1f, 0, 0, 0, 0), 1000.0);
	addTf(*buffer, "base_link", "cam1", rtabmap::Transform(0.1f, -0.1f, 0, 0, 0, 0), 1000.0);
	addOdomMotion(*buffer);                                        // x=0 @1000, x=1 @1001
	geometry_msgs::msg::TransformStamped m;                        // extend to x=2 @1002
	m.header.frame_id = "odom";
	m.child_frame_id = "base_link";
	m.transform.rotation.w = 1.0;
	m.header.stamp = timestampToROS(1002.0);
	m.transform.translation.x = 2.0;
	ASSERT_TRUE(buffer->setTransform(m, "unit_test", false));

	const cv::Mat rgb0(8, 8, CV_8UC3, cv::Scalar(10, 0, 0));
	const cv::Mat rgb1(8, 8, CV_8UC3, cv::Scalar(0, 20, 0));
	const cv::Mat depth0(8, 8, CV_16UC1, cv::Scalar(1000));
	const cv::Mat depth1(8, 8, CV_16UC1, cv::Scalar(2000));

	const std::vector<cv_bridge::CvImageConstPtr> images = {
			makeImage("cam0", 1001.0, rgb0, "bgr8"),
			makeImage("cam1", 1002.0, rgb1, "bgr8")};
	const std::vector<cv_bridge::CvImageConstPtr> depths = {
			makeImage("cam0", 1001.0, depth0, "16UC1"),
			makeImage("cam1", 1002.0, depth1, "16UC1")};
	const std::vector<sensor_msgs::msg::CameraInfo> infos = {
			makeCameraInfo("cam0", 1001.0, 8, 8),
			makeCameraInfo("cam1", 1002.0, 8, 8)};

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	ASSERT_TRUE(convertRGBDMsgs(images, depths, infos, {}, "base_link", "odom",
			timestampToROS(1000.0), rgb, depth, models, stereoModels, *buffer, 0.0, true));

	ASSERT_EQ(models.size(), 2u);
	// cam0 is 1 s after odomStamp, cam1 is 2 s after.
	EXPECT_NEAR(models[0].localTransform().x(), 1.1, 1e-3) << "0.1 + 1.0 of motion";
	EXPECT_NEAR(models[1].localTransform().x(), 2.1, 1e-3) << "0.1 + 2.0 of motion";
	// The corrections must differ, which is the whole point of per-camera stamps.
	EXPECT_GT(models[1].localTransform().x() - models[0].localTransform().x(), 0.5);
	// The lateral offsets are untouched by a purely forward motion.
	EXPECT_NEAR(models[0].localTransform().y(), 0.1, 1e-3);
	EXPECT_NEAR(models[1].localTransform().y(), -0.1, 1e-3);
}

TEST(MsgConversion, convertRGBDMsgsPrefersTheDepthStampWhenTheyDiffer)
{
	// The RGB and depth stamps of a camera are assumed to be equal. This pins the
	// tie-break for when they are not: the depth stamp prevails, since it is the one the
	// geometry is synchronized to. Not a behaviour to rely on -- a camera whose two
	// stamps disagree is already outside the contract.
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "camera_link", rtabmap::Transform(0.1f, 0, 0, 0, 0, 0), 1000.0);
	addOdomMotion(*buffer);

	const cv::Mat rgbImage(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
	const cv::Mat depthImage(8, 8, CV_16UC1, cv::Scalar(1500));

	// RGB stamped at odomStamp (no motion), depth stamped 1 s later (1 m of motion).
	const std::vector<cv_bridge::CvImageConstPtr> images =
			{makeImage("camera_link", 1000.0, rgbImage, "bgr8")};
	const std::vector<cv_bridge::CvImageConstPtr> depths =
			{makeImage("camera_link", 1001.0, depthImage, "16UC1")};
	const std::vector<sensor_msgs::msg::CameraInfo> infos =
			{makeCameraInfo("camera_link", 1000.0, 8, 8)};

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	ASSERT_TRUE(convertRGBDMsgs(images, depths, infos, {}, "base_link", "odom",
			timestampToROS(1000.0), rgb, depth, models, stereoModels, *buffer, 0.0, true));

	ASSERT_EQ(models.size(), 1u);
	EXPECT_NEAR(models[0].localTransform().x(), 1.1, 1e-3)
		<< "the depth stamp (1001) prevails over the rgb stamp (1000)";
}

TEST(MsgConversion, convertRGBDMsgsMultiStereoBuildsOneModelPerPair)
{
	// mono8 "right" images make convertRGBDMsgs take the stereo branch and produce
	// StereoCameraModels instead of CameraModels. The odometry sync is not re-tested
	// here: it happens in the shared loop before the depth/stereo split, so
	// convertRGBDMsgsSyncsEachCameraAtItsOwnStamp already covers it for both.
	const double fx = 100.0;
	const double baseline0 = 0.15;
	const double baseline1 = 0.20;

	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "left0", rtabmap::Transform(0.1f, 0.1f, 0, 0, 0, 0), 1000.0);
	addTf(*buffer, "base_link", "left1", rtabmap::Transform(0.1f, -0.1f, 0, 0, 0, 0), 1000.0);

	const cv::Mat left0(8, 8, CV_8UC1, cv::Scalar(40));
	const cv::Mat left1(8, 8, CV_8UC1, cv::Scalar(60));
	const cv::Mat right(8, 8, CV_8UC1, cv::Scalar(50));

	const std::vector<cv_bridge::CvImageConstPtr> images = {
			makeImage("left0", 1000.0, left0, "mono8"),
			makeImage("left1", 1000.0, left1, "mono8")};
	const std::vector<cv_bridge::CvImageConstPtr> rights = {
			makeImage("right0", 1000.0, right, "mono8"),
			makeImage("right1", 1000.0, right, "mono8")};
	const std::vector<sensor_msgs::msg::CameraInfo> leftInfos = {
			makeCameraInfo("left0", 1000.0, 8, 8, 0.0, fx),
			makeCameraInfo("left1", 1000.0, 8, 8, 0.0, fx)};
	const std::vector<sensor_msgs::msg::CameraInfo> rightInfos = {
			makeCameraInfo("right0", 1000.0, 8, 8, -fx*baseline0, fx),
			makeCameraInfo("right1", 1000.0, 8, 8, -fx*baseline1, fx)};

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	ASSERT_TRUE(convertRGBDMsgs(images, rights, leftInfos, rightInfos, "base_link", "",
			timestampToROS(1000.0), rgb, depth, models, stereoModels, *buffer, 0.0, true));

	EXPECT_TRUE(models.empty()) << "mono8 right images must give stereo models";
	ASSERT_EQ(stereoModels.size(), 2u);

	// Each pair keeps its own baseline and its own local transform.
	EXPECT_NEAR(stereoModels[0].baseline(), baseline0, 1e-6);
	EXPECT_NEAR(stereoModels[1].baseline(), baseline1, 1e-6);
	EXPECT_NEAR(stereoModels[0].localTransform().y(), 0.1, 1e-3);
	EXPECT_NEAR(stereoModels[1].localTransform().y(), -0.1, 1e-3);

	// The two left images are laid out side by side, as in the RGB-D case.
	EXPECT_EQ(rgb.cols, 16);
	EXPECT_EQ(depth.cols, 16);
}

TEST(MsgConversion, convertRGBDMsgsSurvivesAFailedOdomLookup)
{
	// A missing odom frame must only warn: the data is still converted, uncorrected.
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "camera_link", rtabmap::Transform(0.1f, 0, 0.2f, 0, 0, 0), 1001.0);

	const cv::Mat rgbImage(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
	const std::vector<cv_bridge::CvImageConstPtr> images =
			{makeImage("camera_link", 1001.0, rgbImage, "bgr8")};
	const std::vector<sensor_msgs::msg::CameraInfo> infos =
			{makeCameraInfo("camera_link", 1001.0, 8, 8)};

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	ASSERT_TRUE(convertRGBDMsgs(images, {}, infos, {}, "base_link", "odom",
			timestampToROS(1000.0), rgb, depth, models, stereoModels, *buffer, 0.0, true))
		<< "a failed odometry correction must not be fatal";
	ASSERT_EQ(models.size(), 1u);
	EXPECT_NEAR(models[0].localTransform().x(), 0.1, 1e-3) << "left uncorrected";
}

TEST(MsgConversion, convertStereoMsgSyncsToOdomStamp)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "left_link", rtabmap::Transform(0.1f, 0, 0.2f, 0, 0, 0), 1000.0);
	addOdomMotion(*buffer);

	const cv::Mat mono(8, 8, CV_8UC1, cv::Scalar(40));

	cv::Mat left, right;
	rtabmap::StereoCameraModel model;
	ASSERT_TRUE(convertStereoMsg(
			makeImage("left_link", 1001.0, mono, "mono8"),
			makeImage("right_link", 1001.0, mono, "mono8"),
			makeCameraInfo("left_link", 1001.0, 8, 8, 0.0),
			makeCameraInfo("right_link", 1001.0, 8, 8, -15.0),
			"base_link", "odom", timestampToROS(1000.0),
			left, right, model, *buffer, 0.0, true));

	EXPECT_NEAR(model.localTransform().x(), 1.1, 1e-3);
}

TEST(MsgConversion, convertScan3dMsgSyncsToOdomStamp)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "lidar", rtabmap::Transform(0.0f, 0, 0.5f, 0, 0, 0), 1000.0);
	addOdomMotion(*buffer);

	sensor_msgs::msg::PointCloud2 msg = makeXYZCloud({{1.0f, 0.0f, 0.0f}});
	msg.header.stamp = timestampToROS(1001.0);
	msg.header.frame_id = "lidar";

	rtabmap::LaserScan scan;
	ASSERT_TRUE(convertScan3dMsg(msg, "base_link", "odom", timestampToROS(1000.0),
			scan, *buffer, 0.0));

	EXPECT_NEAR(scan.localTransform().x(), 1.0, 1e-3) << "0.0 base->lidar plus 1.0 motion";
	EXPECT_NEAR(scan.localTransform().z(), 0.5, 1e-3);
}

TEST(MsgConversion, convertScanMsgSyncsToOdomStamp)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "laser", rtabmap::Transform(0.2f, 0, 0.1f, 0, 0, 0), 1000.0);
	addOdomMotion(*buffer);

	sensor_msgs::msg::LaserScan msg;
	msg.header.stamp = timestampToROS(1001.0);
	msg.header.frame_id = "laser";
	msg.angle_min = -1.0f;
	msg.angle_max = 1.0f;
	msg.angle_increment = 0.1f;
	msg.time_increment = 0.0f;
	msg.range_min = 0.1f;
	msg.range_max = 30.0f;
	msg.ranges.assign(21, 5.0f);

	rtabmap::LaserScan scan;
	ASSERT_TRUE(convertScanMsg(msg, "base_link", "odom", timestampToROS(1000.0),
			scan, *buffer, 0.0));

	EXPECT_NEAR(scan.localTransform().x(), 1.2, 1e-3) << "0.2 base->laser plus 1.0 motion";
}

TEST(MsgConversion, convertRGBDMsgsRejectsBadEncoding)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "camera_link", rtabmap::Transform::getIdentity(), 1000.0);

	// 32FC1 is a valid depth encoding but not a valid rgb/left one.
	const cv::Mat bad(8, 8, CV_32FC1, cv::Scalar(1.0f));
	const std::vector<cv_bridge::CvImageConstPtr> images =
			{makeImage("camera_link", 1000.0, bad, "32FC1")};
	const std::vector<sensor_msgs::msg::CameraInfo> infos =
			{makeCameraInfo("camera_link", 1000.0, 8, 8)};

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	EXPECT_FALSE(convertRGBDMsgs(images, {}, infos, {}, "base_link", "",
			timestampToROS(1000.0), rgb, depth, models, stereoModels,
			*buffer, 0.0, true));
}

TEST(MsgConversion, convertRGBDMsgsFailsWithoutTf)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();   // empty

	const cv::Mat rgbImage(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
	const std::vector<cv_bridge::CvImageConstPtr> images =
			{makeImage("camera_link", 1000.0, rgbImage, "bgr8")};
	const std::vector<sensor_msgs::msg::CameraInfo> infos =
			{makeCameraInfo("camera_link", 1000.0, 8, 8)};

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	EXPECT_FALSE(convertRGBDMsgs(images, {}, infos, {}, "base_link", "",
			timestampToROS(1000.0), rgb, depth, models, stereoModels,
			*buffer, 0.0, true));
}

TEST(MsgConversion, convertRGBDMsgsCarriesLocalFeatures)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "camera_link", rtabmap::Transform::getIdentity(), 1000.0);

	const cv::Mat rgbImage(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
	const cv::Mat depthImage(8, 8, CV_16UC1, cv::Scalar(1500));
	const std::vector<cv_bridge::CvImageConstPtr> images =
			{makeImage("camera_link", 1000.0, rgbImage, "bgr8")};
	const std::vector<cv_bridge::CvImageConstPtr> depths =
			{makeImage("camera_link", 1000.0, depthImage, "16UC1")};
	const std::vector<sensor_msgs::msg::CameraInfo> infos =
			{makeCameraInfo("camera_link", 1000.0, 8, 8)};

	std::vector<rtabmap_msgs::msg::KeyPoint> kptMsgs(2);
	kptMsgs[0].pt.x = 1.0f; kptMsgs[0].pt.y = 2.0f; kptMsgs[0].size = 7.0f;
	kptMsgs[1].pt.x = 3.0f; kptMsgs[1].pt.y = 4.0f; kptMsgs[1].size = 7.0f;
	std::vector<rtabmap_msgs::msg::Point3f> ptMsgs(2);
	ptMsgs[0].x = 1.0f; ptMsgs[1].x = 2.0f;
	cv::Mat descriptors = cv::Mat::ones(2, 4, CV_32FC1);

	std::vector<cv::KeyPoint> outKpts;
	std::vector<cv::Point3f> outPts;
	cv::Mat outDescriptors;

	cv::Mat rgb, depth;
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::StereoCameraModel> stereoModels;
	ASSERT_TRUE(convertRGBDMsgs(images, depths, infos, {}, "base_link", "",
			timestampToROS(1000.0), rgb, depth, models, stereoModels,
			*buffer, 0.0, true,
			{kptMsgs}, {ptMsgs}, {descriptors},
			&outKpts, &outPts, &outDescriptors));

	ASSERT_EQ(outKpts.size(), 2u);
	EXPECT_FLOAT_EQ(outKpts[0].pt.x, 1.0f);
	EXPECT_FLOAT_EQ(outKpts[1].pt.x, 3.0f);
	ASSERT_EQ(outPts.size(), 2u);
	EXPECT_FLOAT_EQ(outPts[1].x, 2.0f);
	EXPECT_EQ(outDescriptors.rows, 2);
}

TEST(MsgConversion, convertStereoMsgProducesAStereoModel)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	const rtabmap::Transform baseToLeft(0.1f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f);
	addTf(*buffer, "base_link", "left_link", baseToLeft, 1000.0);

	const double fx = 100.0;
	const double baseline = 0.15;
	const cv::Mat leftImage(8, 8, CV_8UC1, cv::Scalar(40));
	const cv::Mat rightImage(8, 8, CV_8UC1, cv::Scalar(50));

	cv::Mat left, right;
	rtabmap::StereoCameraModel model;
	ASSERT_TRUE(convertStereoMsg(
			makeImage("left_link", 1000.0, leftImage, "mono8"),
			makeImage("right_link", 1000.0, rightImage, "mono8"),
			makeCameraInfo("left_link", 1000.0, 8, 8, /*tx=*/0.0, fx),
			makeCameraInfo("right_link", 1000.0, 8, 8, /*tx=*/-fx*baseline, fx),
			"base_link", "", timestampToROS(1000.0),
			left, right, model, *buffer, 0.0, /*alreadyRectified=*/true));

	EXPECT_NEAR(model.baseline(), baseline, 1e-6);
	EXPECT_NEAR(model.left().fx(), fx, 1e-9);
	expectTransformNear(model.localTransform(), baseToLeft, 1e-4f);

	ASSERT_EQ(left.type(), CV_8UC1);
	ASSERT_EQ(right.type(), CV_8UC1);
	EXPECT_EQ(left.at<unsigned char>(0, 0), 40);
	EXPECT_EQ(right.at<unsigned char>(0, 0), 50);
}

TEST(MsgConversion, convertStereoMsgConvertsColourToMono)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "left_link", rtabmap::Transform::getIdentity(), 1000.0);

	const cv::Mat colour(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
	const cv::Mat mono(8, 8, CV_8UC1, cv::Scalar(50));

	cv::Mat left, right;
	rtabmap::StereoCameraModel model;
	ASSERT_TRUE(convertStereoMsg(
			makeImage("left_link", 1000.0, colour, "bgr8"),
			makeImage("right_link", 1000.0, mono, "mono8"),
			makeCameraInfo("left_link", 1000.0, 8, 8, 0.0),
			makeCameraInfo("right_link", 1000.0, 8, 8, -15.0),
			"base_link", "", timestampToROS(1000.0),
			left, right, model, *buffer, 0.0, true));

	// The left image is kept in colour; the right is always reduced to mono.
	EXPECT_EQ(left.type(), CV_8UC3);
	EXPECT_EQ(right.type(), CV_8UC1);
}

TEST(MsgConversion, convertStereoMsgRejectsBadEncoding)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();
	addTf(*buffer, "base_link", "left_link", rtabmap::Transform::getIdentity(), 1000.0);

	const cv::Mat bad(8, 8, CV_32FC1, cv::Scalar(1.0f));
	const cv::Mat mono(8, 8, CV_8UC1, cv::Scalar(50));

	cv::Mat left, right;
	rtabmap::StereoCameraModel model;
	EXPECT_FALSE(convertStereoMsg(
			makeImage("left_link", 1000.0, bad, "32FC1"),
			makeImage("right_link", 1000.0, mono, "mono8"),
			makeCameraInfo("left_link", 1000.0, 8, 8, 0.0),
			makeCameraInfo("right_link", 1000.0, 8, 8, -15.0),
			"base_link", "", timestampToROS(1000.0),
			left, right, model, *buffer, 0.0, true));
}

TEST(MsgConversion, convertStereoMsgFailsWithoutTf)
{
	const std::shared_ptr<tf2_ros::Buffer> buffer = makeTfBuffer();   // empty
	const cv::Mat mono(8, 8, CV_8UC1, cv::Scalar(50));

	cv::Mat left, right;
	rtabmap::StereoCameraModel model;
	EXPECT_FALSE(convertStereoMsg(
			makeImage("left_link", 1000.0, mono, "mono8"),
			makeImage("right_link", 1000.0, mono, "mono8"),
			makeCameraInfo("left_link", 1000.0, 8, 8, 0.0),
			makeCameraInfo("right_link", 1000.0, 8, 8, -15.0),
			"base_link", "", timestampToROS(1000.0),
			left, right, model, *buffer, 0.0, true));
}

/////////////////////////
// IMU
/////////////////////////

TEST(MsgConversion, imuRoundTrip)
{
	sensor_msgs::msg::Imu in;
	in.orientation.x = 0.0;
	in.orientation.y = 0.0;
	in.orientation.z = 0.0;
	in.orientation.w = 1.0;
	in.angular_velocity.x = 0.1;
	in.angular_velocity.y = 0.2;
	in.angular_velocity.z = 0.3;
	in.linear_acceleration.x = 1.0;
	in.linear_acceleration.y = 2.0;
	in.linear_acceleration.z = 9.81;
	for(size_t i=0; i<9; ++i)
	{
		in.orientation_covariance[i] = 0.01 * (i + 1);
		in.angular_velocity_covariance[i] = 0.02 * (i + 1);
		in.linear_acceleration_covariance[i] = 0.03 * (i + 1);
	}

	const rtabmap::IMU imu = imuFromROS(in, rtabmap::Transform::getIdentity());

	sensor_msgs::msg::Imu out;
	imuToROS(imu, out);

	EXPECT_DOUBLE_EQ(out.orientation.w, in.orientation.w);
	EXPECT_DOUBLE_EQ(out.angular_velocity.x, in.angular_velocity.x);
	EXPECT_DOUBLE_EQ(out.angular_velocity.y, in.angular_velocity.y);
	EXPECT_DOUBLE_EQ(out.angular_velocity.z, in.angular_velocity.z);
	EXPECT_DOUBLE_EQ(out.linear_acceleration.x, in.linear_acceleration.x);
	EXPECT_DOUBLE_EQ(out.linear_acceleration.y, in.linear_acceleration.y);
	EXPECT_DOUBLE_EQ(out.linear_acceleration.z, in.linear_acceleration.z);
	for(size_t i=0; i<9; ++i)
	{
		EXPECT_DOUBLE_EQ(out.orientation_covariance[i], in.orientation_covariance[i])
			<< "orientation covariance at " << i;
		EXPECT_DOUBLE_EQ(out.angular_velocity_covariance[i], in.angular_velocity_covariance[i])
			<< "angular velocity covariance at " << i;
		EXPECT_DOUBLE_EQ(out.linear_acceleration_covariance[i], in.linear_acceleration_covariance[i])
			<< "linear acceleration covariance at " << i;
	}
}
