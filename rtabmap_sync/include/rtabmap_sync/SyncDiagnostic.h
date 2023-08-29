#ifndef INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_
#define INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_

#include "rtabmap/utilite/UStl.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "rtabmap/utilite/ULogger.h"

namespace rtabmap_sync {

class SyncDiagnostic {
    public:
        SyncDiagnostic(double tolerance = 0.1, int windowSize = 5) :
            frequencyStatus_(diagnostic_updater::FrequencyStatusParam(&targetFrequency_, &targetFrequency_, tolerance)),
            lastCallbackCalledStamp_(ros::Time::now().toSec()-1),
            targetFrequency_(0.0),
            windowSize_(windowSize)
    {
        UASSERT(windowSize_ >= 1);
    }

protected:
    void initDiagnostic(
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
        diagnosticUpdater_.add(frequencyStatus_);
        for(size_t i=0; i<otherTasks.size(); ++i)
        {
            diagnosticUpdater_.add(*otherTasks[i]);
        }
        diagnosticUpdater_.setHardwareID(strList.empty()?"none":uJoin(strList, "/"));
        diagnosticUpdater_.force_update();
        diagnosticTimer_ = ros::NodeHandle().createTimer(ros::Duration(1), &SyncDiagnostic::diagnosticTimerCallback, this);
    }

    void tick(const ros::Time & stamp, double targetFrequency = 0)
    {
        frequencyStatus_.tick();
        double singlePeriod = stamp.toSec() - lastCallbackCalledStamp_;

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
        lastCallbackCalledStamp_ = stamp.toSec();
    }

private:
    void diagnosticTimerCallback(const ros::TimerEvent& event)
    {
        diagnosticUpdater_.update();

        if(ros::Time::now().toSec()-lastCallbackCalledStamp_ >= 5 && !topicsNotReceivedWarningMsg_.empty())
        {
            ROS_WARN_THROTTLE(5, topicsNotReceivedWarningMsg_.c_str());
        }
    }

    private:
        std::string topicsNotReceivedWarningMsg_;
        diagnostic_updater::Updater diagnosticUpdater_;
        diagnostic_updater::FrequencyStatus frequencyStatus_;
        ros::Timer diagnosticTimer_;
        double lastCallbackCalledStamp_;
        double targetFrequency_;
        int windowSize_;
        std::deque<double> window_;

};

}

#endif /* INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_ */
