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

TEST(MsgConversion, deskewWithoutTimeFieldFails)
{
	// Deskewing needs a per-point time field; a plain XYZ cloud cannot be deskewed.
	const sensor_msgs::msg::PointCloud2 in = makeXYZCloud({{1.0f, 0.0f, 0.0f}});

	sensor_msgs::msg::PointCloud2 out;
	EXPECT_FALSE(deskew(in, out, 0.0, rtabmap::Transform(1, 0, 0, 0, 0, 0)));
}

TEST(MsgConversion, deskewNullVelocityFails)
{
	const sensor_msgs::msg::PointCloud2 in = makeXYZCloud({{1.0f, 0.0f, 0.0f}});

	sensor_msgs::msg::PointCloud2 out;
	EXPECT_FALSE(deskew(in, out, 0.0, rtabmap::Transform()))
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
