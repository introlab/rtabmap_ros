/*
 * MsgConversion.h
 *
 *  Created on: 2012-05-27
 *      Author: mathieu
 */

#ifndef MSGCONVERSION_H_
#define MSGCONVERSION_H_

#include <rtabmap/core/Sensor.h>
#include <rtabmap/core/Actuator.h>
#include "rtabmap/CvMatMsg.h"
#include "rtabmap/SensorMsg.h"
#include "rtabmap/ActuatorMsg.h"
#include <opencv2/highgui/highgui.hpp>

using namespace rtabmap;

cv::Mat fromCvMatMsgToCvMat(const CvMatMsg & matrixMsg)
{
	cv::Mat data;
	if(matrixMsg.compressed)
	{
		data = cv::imdecode(matrixMsg.data, -1);
		if(data.cols != (int)matrixMsg.width || data.rows != (int)matrixMsg.height)
		{
			ROS_ERROR("Uncompressed size (%d/%d) is not %d/%d", data.cols, data.rows, matrixMsg.width, matrixMsg.height);
			data = cv::Mat();
		}
	}
	else
	{
		data = cv::Mat(matrixMsg.height, matrixMsg.width, matrixMsg.dataType);
		if(data.total() * data.elemSize() != matrixMsg.data.size())
		{
			ROS_ERROR("Size attributes (total size=%d) is not equal to actual data size (%d)", data.total() * data.elemSize(), matrixMsg.data.size());
			data = cv::Mat();
		}
		else
		{
			memcpy(data.data, matrixMsg.data.data(), matrixMsg.data.size());
		}
	}
	return data;
}

void fromCvMatToCvMatMsg(CvMatMsg & matrixMsg, const cv::Mat & matrix, bool compressImage = false)
{
	matrixMsg.width = matrix.cols;
	matrixMsg.height = matrix.rows;
	matrixMsg.dataType = matrix.type();
	if(compressImage)
	{
		// compress images
		matrixMsg.compressed = true;
		cv::imencode(".png", matrix, matrixMsg.data);
	}
	else
	{
		matrixMsg.data.resize(matrix.total()*matrix.elemSize());
		memcpy(matrixMsg.data.data(), matrix.data, matrixMsg.data.size());
		matrixMsg.compressed = false;
	}
}


#endif /* MSGCONVERSION_H_ */
