/*
 * OrbitOrientedViewController.h
 *
 *  Created on: 2014-07-13
 *      Author: mathieu
 */

#ifndef ORBITORIENTEDVIEWCONTROLLER_H_
#define ORBITORIENTEDVIEWCONTROLLER_H_

#include "rviz/default_plugin/view_controllers/orbit_view_controller.h"

namespace rtabmap
{

class OrbitOrientedViewController: public rviz::OrbitViewController
{
Q_OBJECT
public:
	OrbitOrientedViewController() {}
	virtual ~OrbitOrientedViewController() {}

protected:
  virtual void updateCamera();
};

}



#endif /* ORBITORIENTEDVIEWCONTROLLER_H_ */
