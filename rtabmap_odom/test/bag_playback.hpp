/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_ODOM_BAG_PLAYBACK_HPP_
#define RTABMAP_ODOM_BAG_PLAYBACK_HPP_

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <string>
#include <vector>

#include "test_data.hpp"

/**
 * @file
 * @brief Reads recorded messages out of a bag, for tests that need real sensor input.
 *
 * The messages are read and replayed by the test itself rather than by `ros2 bag play`:
 * no second process, no wall-clock pacing, and the test controls exactly when each
 * message reaches the node. What the recording provides is the part that cannot be
 * written by hand -- a real driver's cloud layout and a dense TF history around it.
 */

namespace rtabmap_odom_test {

/**
 * @brief The Ouster recording in test/data/lidar; see that directory's README.
 *
 * Two sweeps half a mast turn apart, from a platform that never moves: the only thing
 * between them is the mast's rotation, which TF describes in full.
 */
inline std::string ousterHalfTurnBag()
{
	return testDataRoot() + "/lidar/ouster_half_turn";
}

/**
 * @brief Every message recorded on @p topic, deserialized.
 *
 * Returns an empty vector if the bag or the topic is missing, which the caller is
 * expected to assert on -- a silently empty fixture would make a test pass for the wrong
 * reason.
 */
template <typename MsgT>
std::vector<MsgT> readBagMessages(const std::string & bagPath, const std::string & topic)
{
	std::vector<MsgT> messages;
	rosbag2_cpp::Reader reader;
	try
	{
		reader.open(bagPath);
	}
	catch(const std::exception & e)
	{
		return messages;
	}

	rclcpp::Serialization<MsgT> serialization;
	while(reader.has_next())
	{
		const std::shared_ptr<rosbag2_storage::SerializedBagMessage> message = reader.read_next();
		if(message->topic_name != topic)
		{
			continue;
		}
		rclcpp::SerializedMessage serialized(*message->serialized_data);
		MsgT deserialized;
		serialization.deserialize_message(&serialized, &deserialized);
		messages.push_back(deserialized);
	}
	return messages;
}

}  // namespace rtabmap_odom_test

#endif /* RTABMAP_ODOM_BAG_PLAYBACK_HPP_ */
