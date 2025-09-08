#ifndef INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_
#define INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_

#include "rtabmap/utilite/UStl.h"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "rtabmap_conversions/MsgConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UMutex.h"

using namespace std::chrono_literals;

namespace rtabmap_sync {

class SyncDiagnostic {
    public:
        SyncDiagnostic(rclcpp::Node * node, double tolerance = 0.2, int windowSize = 5) :
		node_(node),
		diagnosticUpdater_(node, 2.0),
		inFrequencyStatus_(diagnostic_updater::FrequencyStatusParam(&inTargetFrequency_, &inTargetFrequency_, tolerance), node->get_clock()),
		inTimeStampStatus_(diagnostic_updater::TimeStampStatusParam(), node->get_clock()),
        outFrequencyStatus_(diagnostic_updater::FrequencyStatusParam(&outTargetFrequency_, &outTargetFrequency_, tolerance), node->get_clock()),
		outTimeStampStatus_(diagnostic_updater::TimeStampStatusParam(), node->get_clock()),
		inCompositeTask_("Input Status"),
        outCompositeTask_("Output Status"),
        lastTickInputStamp_(rtabmap_conversions::timestampFromROS(node_->now())-1),
        inTargetFrequency_(0.0),
		outTargetFrequency_(0.0),
		windowSize_(windowSize),
        lastTickTime_(0.0)
    {
        UASSERT(windowSize_ >= 1);
    }

    void init(
        const std::string & topic,
        const std::string & topicsNotReceivedWarningMsg,
        std::vector<diagnostic_updater::DiagnosticTask*> otherTasks = std::vector<diagnostic_updater::DiagnosticTask*>())
    {
        topicsNotReceivedWarningMsg_ = topicsNotReceivedWarningMsg;

        std::list<std::string> strList = uSplit(topic, '/');
        for(int i=0; i<2 && strList.size()>1; ++i)
        {
            // Assuming format is /back_camera/left/image, we want "back_camera"
            strList.pop_back();
        }
        inCompositeTask_.addTask(&inFrequencyStatus_);
        inCompositeTask_.addTask(&inTimeStampStatus_);
        diagnosticUpdater_.add(inCompositeTask_);
        outCompositeTask_.addTask(&outFrequencyStatus_);
        outCompositeTask_.addTask(&outTimeStampStatus_);
        diagnosticUpdater_.add(outCompositeTask_);
        for(size_t i=0; i<otherTasks.size(); ++i)
        {
            diagnosticUpdater_.add(*otherTasks[i]);
        }
        diagnosticUpdater_.setHardwareID(strList.empty()?"none":uJoin(strList, "/"));
        diagnosticUpdater_.force_update();
        diagnosticTimer_ = node_->create_wall_timer(5s, std::bind(&SyncDiagnostic::diagnosticTimerCallback, this), nullptr);
    }

    void tickInput(const rclcpp::Time & stamp, double expectedFrequency = 0.0)
    {
        updateFrequency(
            stamp,
            expectedFrequency,
            inFrequencyStatus_,
            inTimeStampStatus_,
            inWindow_,
            inTargetFrequency_,
            lastTickInputStamp_);
    }

    void tickOutput(const rclcpp::Time & stamp, double expectedFrequency = 0.0)
    {
        if(expectedFrequency == 0.0) {
            outTargetFrequency_ = inTargetFrequency_;
        }
        double lastTickOutputStamp = 0.0;
        updateFrequency(
            stamp,
            expectedFrequency,
            outFrequencyStatus_,
            outTimeStampStatus_,
            outWindow_,
            outTargetFrequency_,
            lastTickOutputStamp);
    }

private:
    void diagnosticTimerCallback()
    {
        UScopeMutex lock(tickMutex_);
        if(rtabmap_conversions::timestampFromROS(node_->now())-lastTickInputStamp_ >= 5 && !topicsNotReceivedWarningMsg_.empty())
        {
        	RCLCPP_WARN(node_->get_logger(), "%s", topicsNotReceivedWarningMsg_.c_str());
        }
    }

    void updateFrequency(
        const rclcpp::Time & stamp,
        const double & expectedFrequency,
        diagnostic_updater::FrequencyStatus & freqStatus,
        diagnostic_updater::TimeStampStatus & timeStatus,
        std::deque<double> & window,
        double & targetFrequency,
        double & lastTickStamp)
    {
        UScopeMutex lock(tickMutex_);

        freqStatus.tick();
		timeStatus.tick(stamp);

        double stampSec = rtabmap_conversions::timestampFromROS(stamp);

        if(expectedFrequency>0)
        {
            targetFrequency = expectedFrequency;
        }
        else if(lastTickStamp > 0.0) {
            double singlePeriod = stampSec - lastTickStamp;

            window.push_back(singlePeriod);
            if(window.size() > windowSize_)
            {
                window.pop_front();

                double period = 0.0;
                if(window.size() == windowSize_)
                {
                    for(size_t i=0; i<window.size(); ++i)
                    {
                        period += window[i];
                    }
                    period /= windowSize_;
                }

                if(period>0.0 && (targetFrequency == 0.0 || period < 1.0/targetFrequency))
                {
                    targetFrequency = 1.0/period;
                }
            }
        }

        lastTickStamp = stampSec;

        double clockNow = rtabmap_conversions::timestampFromROS(node_->now());
        if(lastTickTime_ > clockNow)
        {
            RCLCPP_WARN(node_->get_logger(), "%s: Detected time jump in the past of %f sec, forcing diagnostic update.", 
                node_->get_name(), lastTickTime_ - clockNow);
            inFrequencyStatus_.clear();
            outFrequencyStatus_.clear();
            diagnosticUpdater_.force_update();
            lastTickInputStamp_ = clockNow;
        }
        lastTickTime_ = clockNow;
    }

private:
    rclcpp::Node * node_;
	std::string topicsNotReceivedWarningMsg_;
	diagnostic_updater::Updater diagnosticUpdater_;
    diagnostic_updater::FrequencyStatus inFrequencyStatus_;
	diagnostic_updater::TimeStampStatus inTimeStampStatus_;
	diagnostic_updater::FrequencyStatus outFrequencyStatus_;
	diagnostic_updater::TimeStampStatus outTimeStampStatus_;
	diagnostic_updater::CompositeDiagnosticTask inCompositeTask_;
    diagnostic_updater::CompositeDiagnosticTask outCompositeTask_;
	rclcpp::TimerBase::SharedPtr diagnosticTimer_;
	double lastTickInputStamp_;
	double inTargetFrequency_;
    double outTargetFrequency_;
	int windowSize_;
	std::deque<double> inWindow_;
    std::deque<double> outWindow_;
    UMutex tickMutex_;
    double lastTickTime_;

};

}

#endif /* INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_ */
