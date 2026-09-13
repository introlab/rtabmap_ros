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

/**
 * @brief Reports the rate going into a synchronizer and the rate coming out of it, on
 *        /diagnostics.
 *
 * Every node in this package, and every node built on CommonDataSubscriber, publishes
 * through one of these. Two statuses rather than one is the whole point: a node can be
 * receiving all of its inputs and still publish nothing -- one camera lagging is enough
 * to stop a synchronizer emitting -- and only the pair tells those cases apart.
 *
 * @par Expected rate
 * With no rate given, the target is learned from the gaps between the message stamps,
 * averaged over a sliding window, and only ever revised upwards to the fastest rate seen.
 * A node that deliberately publishes slower than it receives -- a throttled or decimated
 * output -- passes its own rate to tickOutput() instead, so it is judged against what it
 * meant to do.
 *
 * @par Usage
 * @code
 * syncDiagnostic_.reset(new SyncDiagnostic(this));
 * syncDiagnostic_->init(imageSub_.getTopic(), "Did not receive data since 5 seconds!...");
 * // then, in the callback:
 * syncDiagnostic_->tickInput(image->header.stamp);
 * ...
 * syncDiagnostic_->tickOutput(image->header.stamp);
 * @endcode
 *
 * @note The node passed in is held as a raw pointer and must outlive this object.
 */
class SyncDiagnostic {
    public:
        /**
         * @param node       the node to publish /diagnostics from; must outlive this object
         * @param tolerance  fraction by which the measured rate may differ from the
         *                   expected one before the status stops being OK
         * @param windowSize number of stamp intervals averaged when learning the expected
         *                   rate; must be at least 1
         */
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

    /**
     * @brief Registers the tasks and starts publishing.
     *
     * @param topic one of the subscribed topics, used only to name the hardware the
     *              status belongs to: the last two segments are dropped, so
     *              `/back_camera/left/image` reports as `back_camera`. Pass an empty
     *              string when no single topic identifies the device; the hardware id is
     *              then `none`.
     * @param topicsNotReceivedWarningMsg logged every 5 seconds while nothing is coming
     *              in. Worth making specific: it is what a user sees when a pipeline is
     *              silent, so it should name the topics and the likely causes.
     * @param otherTasks extra tasks to publish in the same message, so a node's own state
     *              arrives alongside its rates rather than in a separate update.
     */
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

    /**
     * @brief Records that one input message arrived.
     * @param stamp             the message stamp; it is also checked against the clock,
     *                          which is how an unsynchronized sender is caught
     * @param expectedFrequency the rate to judge against, or 0 to learn it from the stamps
     */
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

    /**
     * @brief Records that one output message was published.
     * @param stamp             the stamp of what was published
     * @param expectedFrequency the rate to judge against, or 0 to inherit the rate
     *                          measured on the input side -- the right default for a
     *                          node that publishes one output per input
     */
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
