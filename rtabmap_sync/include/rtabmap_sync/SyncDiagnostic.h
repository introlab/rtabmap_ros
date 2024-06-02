#ifndef INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_
#define INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_

#include "rtabmap/utilite/UStl.h"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "rtabmap_conversions/MsgConversion.h"
#include "rtabmap/utilite/ULogger.h"

using namespace std::chrono_literals;

namespace rtabmap_sync {

class SyncDiagnostic {
    public:
        SyncDiagnostic(rclcpp::Node * node, double tolerance = 0.1, int windowSize = 5) :
		node_(node),
		diagnosticUpdater_(node),
		frequencyStatus_(diagnostic_updater::FrequencyStatusParam(&targetFrequency_, &targetFrequency_, tolerance), node->get_clock()),
		timeStampStatus_(diagnostic_updater::TimeStampStatusParam(), node->get_clock()),
		compositeTask_("Sync status"),
		lastCallbackCalledStamp_(rtabmap_conversions::timestampFromROS(node_->now())-1),
		targetFrequency_(0.0),
		windowSize_(windowSize)
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
        compositeTask_.addTask(&frequencyStatus_);
        compositeTask_.addTask(&timeStampStatus_);
        diagnosticUpdater_.add(compositeTask_);
        for(size_t i=0; i<otherTasks.size(); ++i)
        {
            diagnosticUpdater_.add(*otherTasks[i]);
        }
        diagnosticUpdater_.setHardwareID(strList.empty()?"none":uJoin(strList, "/"));
        diagnosticUpdater_.force_update();
        diagnosticTimer_ = node_->create_wall_timer(1s, std::bind(&SyncDiagnostic::diagnosticTimerCallback, this), nullptr);
    }

    void tick(const rclcpp::Time & stamp, double targetFrequency = 0)
    {
        frequencyStatus_.tick();
		timeStampStatus_.tick(stamp);
        double singlePeriod = rtabmap_conversions::timestampFromROS(stamp) - lastCallbackCalledStamp_;

        window_.push_back(singlePeriod);
        if(window_.size() > windowSize_)
        {
            window_.pop_front();
        }
        double period = 0.0;
        if(window_.size() == windowSize_)
        {
            for(size_t i=0; i<window_.size(); ++i)
            {
                period += window_[i];
            }
            period /= windowSize_;
        }

        if(period>0.0 && targetFrequency == 0 && (targetFrequency_ == 0.0 || period < 1.0/targetFrequency_))
        {
            targetFrequency_ = 1.0/period;
        }
        else if(targetFrequency>0)
        {
            targetFrequency_ = targetFrequency;
        }
        lastCallbackCalledStamp_ = rtabmap_conversions::timestampFromROS(stamp);
    }

private:
    void diagnosticTimerCallback()
    {
        if(rtabmap_conversions::timestampFromROS(node_->now())-lastCallbackCalledStamp_ >= 5 && !topicsNotReceivedWarningMsg_.empty())
        {
        	RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "%s", topicsNotReceivedWarningMsg_.c_str());
        }
    }

private:
    rclcpp::Node * node_;
	std::string topicsNotReceivedWarningMsg_;
	diagnostic_updater::Updater diagnosticUpdater_;
	diagnostic_updater::FrequencyStatus frequencyStatus_;
	diagnostic_updater::TimeStampStatus timeStampStatus_;
	diagnostic_updater::CompositeDiagnosticTask compositeTask_;
	rclcpp::TimerBase::SharedPtr diagnosticTimer_;
	double lastCallbackCalledStamp_;
	double targetFrequency_;
	int windowSize_;
	std::deque<double> window_;

};

}

#endif /* INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_ */
