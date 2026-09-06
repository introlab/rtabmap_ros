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
