/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

#include <rtabmap/core/Parameters.h>

#include "core_wrapper_fixture.hpp"

namespace rtabmap_slam_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

using rtabmap::Parameters;

class CoreWrapperParametersTest : public CoreWrapperTest
{
protected:
	/// RTAB-Map's own default for @p key, spelled the way the node stores it.
	static std::string rtabmapDefault(const std::string & key)
	{
		return Parameters::getDefaultParameters().at(key);
	}

	static std::string readFile(const std::string & path)
	{
		std::ifstream in(path);
		std::stringstream s;
		s << in.rdbuf();
		return s.str();
	}
};

/**
 * Every RTAB-Map parameter is a ROS parameter under its own name, declared as a string --
 * that is how RTAB-Map's own parameter map stores them, whatever the value looks like.
 * The odometry ones are left out: they belong to the odometry nodes, and declaring them
 * here would suggest that setting them on rtabmap does something.
 */
TEST_F(CoreWrapperParametersTest, declares_rtabmap_parameters_as_strings_except_odometry)
{
	makeNode();

	ASSERT_TRUE(node_->has_parameter(Parameters::kRtabmapDetectionRate()));
	EXPECT_EQ(rclcpp::ParameterType::PARAMETER_STRING,
			node_->get_parameter(Parameters::kRtabmapDetectionRate()).get_type());
	EXPECT_EQ(rclcpp::ParameterType::PARAMETER_STRING,
			node_->get_parameter(Parameters::kMemIncrementalMemory()).get_type());

	EXPECT_FALSE(node_->has_parameter(Parameters::kOdomStrategy()));
	EXPECT_FALSE(node_->has_parameter(Parameters::kOdomResetCountdown()));
	EXPECT_FALSE(node_->has_parameter(Parameters::kOdomF2MMaxSize()));
}

/**
 * Two defaults differ from RTAB-Map's own: the occupancy grid is built by default, since
 * on a robot that is what the map is for, and the working directory is ~/.ros, or
 * $ROS_HOME, instead of RTAB-Map's own.
 */
TEST_F(CoreWrapperParametersTest, builds_the_occupancy_grid_by_default)
{
	ASSERT_FALSE(Parameters::defaultRGBDCreateOccupancyGrid())
			<< "RTAB-Map's own default changed: this test no longer shows a difference";

	makeNode();

	EXPECT_EQ("true", param(Parameters::kRGBDCreateOccupancyGrid()));
	EXPECT_EQ(dir(), param(Parameters::kRtabmapWorkingDirectory()));
}

TEST_F(CoreWrapperParametersTest, applies_rtabmap_parameters_set_as_ros_parameters)
{
	makeNode({rclcpp::Parameter(Parameters::kMemRehearsalSimilarity(), "0.45"),
			  rclcpp::Parameter(Parameters::kRGBDLinearUpdate(), "0.3")});

	EXPECT_EQ("0.45", param(Parameters::kMemRehearsalSimilarity()));
	EXPECT_EQ("0.3", param(Parameters::kRGBDLinearUpdate()));
}

/**
 * The declared type is string, so a value given with its natural type is refused at
 * construction rather than silently converted: the quoting in `-p "Rtabmap/DetectionRate:='2'"`
 * is not optional.
 */
TEST_F(CoreWrapperParametersTest, refuses_a_rtabmap_parameter_given_as_a_number)
{
	rclcpp::NodeOptions options;
	options.parameter_overrides(defaultParameters(
			{rclcpp::Parameter(Parameters::kRGBDLinearUpdate(), 0.3)}));
	EXPECT_ANY_THROW(std::make_shared<rtabmap_slam::CoreWrapper>(options));
}

TEST_F(CoreWrapperParametersTest, applies_rtabmap_parameters_passed_as_arguments)
{
	makeNode({}, {"--Mem/RehearsalSimilarity", "0.21"});

	EXPECT_EQ("0.21", param(Parameters::kMemRehearsalSimilarity()));
}

/**
 * config_path is an INI file of RTAB-Map parameters, read at startup. The odometry
 * parameters in it are ignored, like everywhere else on this node, and ROS parameters
 * set explicitly win over the file.
 */
TEST_F(CoreWrapperParametersTest, loads_parameters_from_config_path)
{
	const std::string ini = dir() + "/config.ini";
	{
		std::ofstream out(ini);
		out << "[Core]\n"
			<< "Mem/RehearsalSimilarity = 0.44\n"
			<< "RGBD/LinearUpdate = 0.7\n"
			<< "Odom/Strategy = 1\n";
	}

	makeNode({rclcpp::Parameter("config_path", ini),
			  rclcpp::Parameter(Parameters::kRGBDLinearUpdate(), "0.2")});

	EXPECT_EQ("0.44", param(Parameters::kMemRehearsalSimilarity()));
	EXPECT_EQ("0.2", param(Parameters::kRGBDLinearUpdate()));
	EXPECT_FALSE(node_->has_parameter(Parameters::kOdomStrategy()));
}

/// The node writes its parameters back to config_path when it shuts down.
TEST_F(CoreWrapperParametersTest, saves_parameters_to_config_path_on_shutdown)
{
	const std::string ini = dir() + "/generated.ini";
	ASSERT_FALSE(UFile::exists(ini));

	makeNode({rclcpp::Parameter("config_path", ini),
			  rclcpp::Parameter(Parameters::kMemRehearsalSimilarity(), "0.37")});
	destroyNode();

	ASSERT_TRUE(UFile::exists(ini));
	rtabmap::ParametersMap saved;
	Parameters::readINI(ini, saved);
	ASSERT_TRUE(saved.count(Parameters::kMemRehearsalSimilarity()));
	EXPECT_EQ("0.37", saved.at(Parameters::kMemRehearsalSimilarity()));
}

/**
 * A database remembers the parameters it was built with, and reopening it without
 * setting them again brings them back: a map made with a given configuration is reopened
 * with that configuration. What is set explicitly still wins.
 */
TEST_F(CoreWrapperParametersTest, reuses_the_parameters_stored_in_the_database)
{
	makeNode({rclcpp::Parameter(Parameters::kMemRehearsalSimilarity(), "0.33"),
			  rclcpp::Parameter(Parameters::kRGBDLinearUpdate(), "0.25")});
	destroyNode();
	ASSERT_TRUE(UFile::exists(databasePath()));

	makeNode({rclcpp::Parameter(Parameters::kRGBDLinearUpdate(), "0.15")});

	EXPECT_EQ("0.33", param(Parameters::kMemRehearsalSimilarity()));
	EXPECT_EQ("0.15", param(Parameters::kRGBDLinearUpdate()));
}

/// delete_db_on_start starts over: a new, empty database, with none of the old parameters.
TEST_F(CoreWrapperParametersTest, delete_db_on_start_forgets_the_stored_parameters)
{
	makeNode({rclcpp::Parameter(Parameters::kMemRehearsalSimilarity(), "0.33")});
	destroyNode();

	makeNode({rclcpp::Parameter("delete_db_on_start", true)});

	EXPECT_EQ(rtabmapDefault(Parameters::kMemRehearsalSimilarity()),
			param(Parameters::kMemRehearsalSimilarity()));
}

/// `-d` and `--delete_db_on_start` as arguments do the same, the form launch files used.
TEST_F(CoreWrapperParametersTest, delete_db_on_start_can_be_passed_as_an_argument)
{
	makeNode({rclcpp::Parameter(Parameters::kMemRehearsalSimilarity(), "0.33")});
	destroyNode();

	makeNode({}, {"-d"});

	EXPECT_EQ(rtabmapDefault(Parameters::kMemRehearsalSimilarity()),
			param(Parameters::kMemRehearsalSimilarity()));
}

/**
 * With no camera subscribed, there is nothing to extract visual words from: bag-of-words
 * loop closure detection is switched off rather than left to fail on every frame.
 */
TEST_F(CoreWrapperParametersTest, odometry_only_input_disables_bag_of_words)
{
	makeNode();

	EXPECT_EQ("-1", param(Parameters::kKpMaxFeatures()));
	EXPECT_EQ(rtabmapDefault(Parameters::kRegStrategy()), param(Parameters::kRegStrategy()));
}

/**
 * With a 2D lidar and no camera, the node reconfigures itself for it: the grid is built
 * from the scan without a range limit, loop closures are registered with ICP, proximity
 * detection merges the last 10 scans, and bag-of-words is off.
 */
TEST_F(CoreWrapperParametersTest, laser_scan_input_switches_to_icp_and_scan_grid)
{
	makeNode({rclcpp::Parameter("subscribe_scan", true)});

	EXPECT_EQ("0", param(Parameters::kGridSensor()));
	EXPECT_EQ("0", param(Parameters::kGridRangeMax()));
	EXPECT_EQ("1", param(Parameters::kRegStrategy()));
	EXPECT_EQ("10", param(Parameters::kRGBDProximityPathMaxNeighbors()));
	EXPECT_EQ("-1", param(Parameters::kKpMaxFeatures()));
}

/// None of those adjustments overrides a value set explicitly.
TEST_F(CoreWrapperParametersTest, laser_scan_adjustments_keep_explicit_values)
{
	makeNode({rclcpp::Parameter("subscribe_scan", true),
			  rclcpp::Parameter(Parameters::kGridSensor(), "1"),
			  rclcpp::Parameter(Parameters::kRGBDProximityPathMaxNeighbors(), "3")});

	EXPECT_EQ("1", param(Parameters::kGridSensor()));
	EXPECT_EQ(rtabmapDefault(Parameters::kGridRangeMax()), param(Parameters::kGridRangeMax()))
			<< "the range limit is only lifted for a grid built from the scan";
	EXPECT_EQ("3", param(Parameters::kRGBDProximityPathMaxNeighbors()));
}

/**
 * A 3D lidar gets the same treatment, with one difference: proximity detection registers
 * against the single nearest scan rather than merging ten.
 */
TEST_F(CoreWrapperParametersTest, scan_cloud_input_switches_to_icp)
{
	makeNode({rclcpp::Parameter("subscribe_scan_cloud", true)});

	EXPECT_EQ("0", param(Parameters::kGridSensor()));
	EXPECT_EQ("1", param(Parameters::kRegStrategy()));
	EXPECT_EQ("1", param(Parameters::kRGBDProximityPathMaxNeighbors()));
	EXPECT_EQ("-1", param(Parameters::kKpMaxFeatures()));
}

/**
 * A cloud flagged as 2D -- a 2D lidar published as a cloud -- is treated like a laser
 * scan, merging ten, whether ICP was selected explicitly or by the node's own switch to it
 * for lack of a camera.
 */
TEST_F(CoreWrapperParametersTest, scan_cloud_flagged_2d_merges_scans_like_a_laser_scan)
{
	makeNode({rclcpp::Parameter("subscribe_scan_cloud", true),
			  rclcpp::Parameter("scan_cloud_is_2d", true),
			  rclcpp::Parameter(Parameters::kRegStrategy(), "1")});
	EXPECT_EQ("10", param(Parameters::kRGBDProximityPathMaxNeighbors())) << "ICP selected explicitly";
	destroyNode();

	makeNode({rclcpp::Parameter("subscribe_scan_cloud", true),
			  rclcpp::Parameter("scan_cloud_is_2d", true),
			  rclcpp::Parameter("delete_db_on_start", true)});
	EXPECT_EQ("1", param(Parameters::kRegStrategy())) << "switched to ICP by the node";
	EXPECT_EQ("10", param(Parameters::kRGBDProximityPathMaxNeighbors()));
}

/**
 * A parameter RTAB-Map has renamed is still honoured under its old name, with a warning,
 * so that an old launch file keeps working. Old names are never declared, so this is only
 * possible by reading them from the overrides.
 */
TEST_F(CoreWrapperParametersTest, migrates_a_renamed_parameter)
{
	// g2o/PixelVariance became Optimizer/PixelVariance.
	ASSERT_TRUE(Parameters::getRemovedParameters().count("g2o/PixelVariance"));
	makeNode({rclcpp::Parameter("g2o/PixelVariance", "2.5")});

	EXPECT_EQ("2.5", param(Parameters::kOptimizerPixelVariance()));
}

}  // namespace

}  // namespace rtabmap_slam_test
