#ifndef INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_
#define INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_

#include "rtabmap/utilite/UStl.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

namespace rtabmap_sync {

class SyncDiagnostic {
    public:
        SyncDiagnostic(double tolerance = 0.1) :
            compositeTask_("Sync Status"),
            frequencyStatus_(diagnostic_updater::FrequencyStatusParam(&targetFrequency_, &targetFrequency_, tolerance)),
            lastCallbackCalledStamp_(ros::Time::now().toSec()-1),
            targetFrequency_(0.0)
    {}

protected:
    void initDiagnostic(
        const std::string & imageTopic,
        const std::string & topicsNotReceivedWarningMsg)
    {
        topicsNotReceivedWarningMsg_ = topicsNotReceivedWarningMsg;

		std::list<std::string> strList = uSplit(imageTopic, '/');
		for(int i=0; i<2 && strList.size()>1; ++i)
		{
			// Assuming format is /back_camera/left/image, we want "back_camera"
			strList.pop_back();
		}
		compositeTask_.addTask(&frequencyStatus_);
		compositeTask_.addTask(&timeStampStatus_);
		diagnosticUpdater_.setHardwareID(strList.empty()?"none":uJoin(strList, "/"));
		diagnosticUpdater_.add(compositeTask_);
		diagnosticUpdater_.force_update();
		diagnosticTimer_ = ros::NodeHandle().createTimer(ros::Duration(1), &SyncDiagnostic::diagnosticTimerCallback, this);
    }

    void tick(const ros::Time & stamp, double targetFrequency = 0)
    {
        frequencyStatus_.tick();
		timeStampStatus_.tick(stamp.toSec());
		double period = stamp.toSec() - lastCallbackCalledStamp_;
		if(period>0.0 && targetFrequency == 0 && (targetFrequency_ == 0.0 || period < 1.0/targetFrequency_))
		{
			targetFrequency_ = 1.0/period;
		}
        else if(targetFrequency>0)
        {
            targetFrequency_ = targetFrequency;
        }
		lastCallbackCalledStamp_ = stamp.toSec();
		diagnosticUpdater_.update();
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
        diagnostic_updater::CompositeDiagnosticTask compositeTask_;
        diagnostic_updater::FrequencyStatus frequencyStatus_;
        diagnostic_updater::SlowTimeStampStatus timeStampStatus_;
        ros::Timer diagnosticTimer_;
        double lastCallbackCalledStamp_;
        double targetFrequency_;

};

}

#endif /* INCLUDE_RTABMAP_SYNC_SYNCDIAGNOSTIC_H_ */