
#ifndef INFO_DISPLAY_H
#define INFO_DISPLAY_H

#include <rtabmap/Info.h>

#include <rviz/message_filter_display.h>

namespace rtabmap
{

class InfoDisplay: public rviz::MessageFilterDisplay<rtabmap::Info>
{
Q_OBJECT
public:
	InfoDisplay();
	virtual ~InfoDisplay();

	virtual void reset();
	virtual void update( float wall_dt, float ros_dt );

protected:
	/** @brief Do initialization. Overridden from MessageFilterDisplay. */
	virtual void onInitialize();

	/** @brief Process a single message.  Overridden from MessageFilterDisplay. */
	virtual void processMessage( const rtabmap::InfoConstPtr& cloud );

private:
	ros::AsyncSpinner spinner_;
	ros::CallbackQueue cbqueue_;

	QString info_;
	int globalCount_;
	int localCount_;
	boost::mutex info_mutex_;
};

} // namespace rtabmap

#endif
