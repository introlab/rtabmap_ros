/*
 * VisualAttentionNode.cpp
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <utilite/UPlot.h>
#include <QtGui/QApplication>
#include <QtGui/QSpinBox>
#include <QtGui/QCheckBox>
#include <QtGui/QVBoxLayout>
#include <opencv2/imgproc/imgproc_c.h>

image_transport::Publisher rosPublisherMotionGlobal;
image_transport::Publisher rosPublisherMotionLocal;
image_transport::Publisher rosPublisherLocal;
image_transport::Publisher rosPublisherMotionLocalPolar;
image_transport::Publisher rosPublisherMotionLocalPolarReconstructed;
image_transport::Publisher rosPublisherLocalPolar;
image_transport::Publisher rosPublisherLocalPolarReconstructed;
image_transport::Publisher rosPublisherWithRoi;
UPlotCurve * g_curveR = 0;
UPlotCurve * g_curveG = 0;
UPlotCurve * g_curveB = 0;

UPlotCurve * g_curveRRatio = 0;
UPlotCurve * g_curveGRatio = 0;
UPlotCurve * g_curveBRatio = 0;

QSpinBox * spinX = 0;
QSpinBox * spinY = 0;
QCheckBox * polarCheckBox = 0;

#define ROI_RATIO 4 // default 8
#define POLAR_RAYS 128
#define POLAR_RINGS 64
cv::Mat previousGlobalImage;
cv::Mat previousPolarROI;
cv::Rect roi;
float ratio = 0.2;
bool attentionDisabled = false; // may set ROI_RATIO=1 if true
bool localAttention = false;

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(msg->data.size())
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);

		if(ptr->image.depth() == CV_8U && ptr->image.channels() == 3)
		{
			cv::Mat motion = ptr->image.clone();
			if(previousGlobalImage.cols == ptr->image.cols && previousGlobalImage.rows == ptr->image.rows)
			{
				unsigned char * imageData = (unsigned char *)motion.data;
				unsigned char * previous_imageData = (unsigned char *)previousGlobalImage.data;
				int widthStep = motion.cols * motion.elemSize();
				cv::Point2i centerROILocal(roi.width/2, roi.height/2);
				cv::Point2i centerROIGlobal(roi.x+roi.width/2, roi.y+roi.height/2);
				cv::Point2i nearestMovingPixel;

				for(int j=0; j<motion.rows; ++j)
				{
					for(int i=0; i<motion.cols; ++i)
					{
						float b = (float)imageData[j*widthStep+i*3+0];
						float g = (float)imageData[j*widthStep+i*3+1];
						float r = (float)imageData[j*widthStep+i*3+2];
						float previous_b = (float)previous_imageData[j*widthStep+i*3+0];
						float previous_g = (float)previous_imageData[j*widthStep+i*3+1];
						float previous_r = (float)previous_imageData[j*widthStep+i*3+2];

						if(!(fabs(b-previous_b)/256.0f>=ratio || fabs(g-previous_g)/256.0f >= ratio || fabs(r-previous_r)/256.0f >= ratio))
						{
							imageData[j*widthStep+i*3+0] = 0;
							imageData[j*widthStep+i*3+1] = 0;
							imageData[j*widthStep+i*3+2] = 0;
						}
					}
				}

				// Motion ROI
				cv::Mat motionROI = cv::Mat(motion, roi);
				if(rosPublisherMotionLocal.getNumSubscribers())
				{
					cv_bridge::CvImage img;
					img.header.frame_id = msg->header.frame_id;
					img.header.stamp = msg->header.stamp;
					img.encoding = sensor_msgs::image_encodings::BGR8;
					img.image = motionROI;
					rosPublisherMotionLocal.publish(img.toImageMsg());
				}

				/*{
					int radius = motionROI.cols/2;
					float M = 64/std::log(radius);
					cv::Mat tmpPolar(128, 64, CV_8UC3);
					IplImage iplTmpPolar = tmpPolar;
					IplImage iplPolar = motionROI;
					cvLogPolar( &iplPolar, &iplTmpPolar, cvPoint2D32f(radius, radius), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
					cvLogPolar( &iplTmpPolar, &iplPolar, cvPoint2D32f(radius, radius), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS+CV_WARP_INVERSE_MAP );

					if(rosPublisherMotionLocalPolar.getNumSubscribers())
					{
						cv_bridge::CvImage img;
						img.encoding = sensor_msgs::image_encodings::BGR8;
						img.image = motionROI;
						rosPublisherMotionLocalPolar.publish(img.toImageMsg());
					}
				}*/


				// Motion ROI polar
				{
					cv::Mat imageROI = cv::Mat(ptr->image, roi);
					int radius = imageROI.cols/2;
					float M = POLAR_RINGS/std::log(radius);
					cv::Mat polarROI(POLAR_RAYS, POLAR_RINGS, CV_8UC3);
					IplImage iplPolar = polarROI;
					IplImage iplImageROI = imageROI;
					cvLogPolar( &iplImageROI, &iplPolar, cvPoint2D32f(radius, radius), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
					cv::Mat motionPolarROI = polarROI.clone();
					if(previousPolarROI.cols == motionPolarROI.cols && previousPolarROI.rows == motionPolarROI.rows)
					{
						unsigned char * motionPolarData = motionPolarROI.data;
						unsigned char * previousPolarData = previousPolarROI.data;
						widthStep = motionPolarROI.cols * motionPolarROI.elemSize();
						for(int j=0; j<motionPolarROI.rows; ++j)
						{
							for(int i=0; i<motionPolarROI.cols; ++i)
							{
								float b = (float)motionPolarData[j*widthStep+i*3+0];
								float g = (float)motionPolarData[j*widthStep+i*3+1];
								float r = (float)motionPolarData[j*widthStep+i*3+2];
								float previous_b = (float)previousPolarData[j*widthStep+i*3+0];
								float previous_g = (float)previousPolarData[j*widthStep+i*3+1];
								float previous_r = (float)previousPolarData[j*widthStep+i*3+2];

								if(!(fabs(b-previous_b)/256.0f>=ratio || fabs(g-previous_g)/256.0f >= ratio || fabs(r-previous_r)/256.0f >= ratio))
								{
									motionPolarData[j*widthStep+i*3+0] = 0;
									motionPolarData[j*widthStep+i*3+1] = 0;
									motionPolarData[j*widthStep+i*3+2] = 0;
								}
							}
						}

						if(rosPublisherLocalPolar.getNumSubscribers())
						{
							cv_bridge::CvImage img;
							img.header.frame_id = msg->header.frame_id;
							img.header.stamp = msg->header.stamp;
							img.encoding = sensor_msgs::image_encodings::BGR8;
							img.image = polarROI;
							rosPublisherLocalPolar.publish(img.toImageMsg());
						}

						if(rosPublisherLocalPolarReconstructed.getNumSubscribers())
						{
							cv::Mat reconstructedROI(imageROI.rows, imageROI.cols, imageROI.type());
							IplImage iplPolarROI = polarROI;
							IplImage iplReconstructedROI = reconstructedROI;
							cvLogPolar( &iplPolarROI, &iplReconstructedROI, cvPoint2D32f(radius, radius), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS+CV_WARP_INVERSE_MAP );

							cv_bridge::CvImage img;
							img.header.frame_id = msg->header.frame_id;
							img.header.stamp = msg->header.stamp;
							img.encoding = sensor_msgs::image_encodings::BGR8;
							img.image = reconstructedROI;
							rosPublisherLocalPolarReconstructed.publish(img.toImageMsg());
						}

						if(rosPublisherMotionLocalPolar.getNumSubscribers())
						{
							cv_bridge::CvImage img;
							img.header.frame_id = msg->header.frame_id;
							img.header.stamp = msg->header.stamp;
							img.encoding = sensor_msgs::image_encodings::BGR8;
							img.image = motionPolarROI;
							rosPublisherMotionLocalPolar.publish(img.toImageMsg());
						}

						//reconstruct image
						cv::Mat reconstructedROI(imageROI.rows, imageROI.cols, imageROI.type());
						IplImage iplMotionPolarROI = motionPolarROI;
						IplImage iplReconstructedROI = reconstructedROI;
						cvLogPolar( &iplMotionPolarROI, &iplReconstructedROI, cvPoint2D32f(radius, radius), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS+CV_WARP_INVERSE_MAP );

						if(rosPublisherMotionLocalPolarReconstructed.getNumSubscribers())
						{
							cv_bridge::CvImage img;
							img.header.frame_id = msg->header.frame_id;
							img.header.stamp = msg->header.stamp;
							img.encoding = sensor_msgs::image_encodings::BGR8;
							img.image = reconstructedROI;
							rosPublisherMotionLocalPolarReconstructed.publish(img.toImageMsg());
						}

						if(localAttention)
						{
							float distNearest = 0.0f;
							unsigned char * reconstructedData = reconstructedROI.data;
							widthStep = reconstructedROI.cols * reconstructedROI.elemSize();
							for(int j=0; j<reconstructedROI.rows; ++j)
							{
								for(int i=0; i<reconstructedROI.cols; ++i)
								{
									if(reconstructedData[j*widthStep+i*3+0] ||
										reconstructedData[j*widthStep+i*3+1] ||
										reconstructedData[j*widthStep+i*3+2])
									{
										float dist = std::sqrt(float((i-centerROILocal.x)*(i-centerROILocal.x) + (j-centerROILocal.y)*(j-centerROILocal.y)));
										if((nearestMovingPixel.x == 0 && nearestMovingPixel.y == 0) ||
											dist < distNearest)
										{
											nearestMovingPixel.y = j;
											nearestMovingPixel.x = i;
											distNearest = dist;
										}
									}
								}
							}
						}

						// PLOT
						if(g_curveR && g_curveG && g_curveB)
						{
							cv::Mat ref = ptr->image;
							if(polarCheckBox->isChecked())
							{
								ref = reconstructedROI;
							}

							if(spinX->value()<0 || spinX->value() > ref.cols)
							{
								spinX->setValue(ref.cols/2);
							}
							if(spinY->value()<0 || spinY->value() > ref.rows)
							{
								spinY->setValue(ref.rows/2);
							}

							//take pixel (OpenCV is BGR)
							float r = (float)*(ref.data + (spinX->value())*ref.elemSize() + (spinY->value())*ref.elemSize()*ref.cols + 2);
							float g = (float)*(ref.data + (spinX->value())*ref.elemSize() + (spinY->value())*ref.elemSize()*ref.cols + 1);
							float b = (float)*(ref.data + (spinX->value())*ref.elemSize() + (spinY->value())*ref.elemSize()*ref.cols + 0);
							if(g_curveR->itemsSize())
							{
								float lastR = g_curveR->getItemData(g_curveR->itemsSize()-1).y();
								float lastG = g_curveG->getItemData(g_curveG->itemsSize()-1).y();
								float lastB = g_curveB->getItemData(g_curveB->itemsSize()-1).y();
								QMetaObject::invokeMethod(g_curveRRatio, "addValue", Q_ARG(float, fabs(r-lastR)/255.0f) );
								QMetaObject::invokeMethod(g_curveGRatio, "addValue", Q_ARG(float, fabs(g-lastG)/255.0f) );
								QMetaObject::invokeMethod(g_curveBRatio, "addValue", Q_ARG(float, fabs(b-lastB)/255.0f) );
							}

							QMetaObject::invokeMethod(g_curveR, "addValue", Q_ARG(float, (float)r));
							QMetaObject::invokeMethod(g_curveG, "addValue", Q_ARG(float, (float)g));
							QMetaObject::invokeMethod(g_curveB, "addValue", Q_ARG(float, (float)b));
						}
					}

				}

				if(!localAttention)
				{
					//global, may be outside of the ROI
					float distNearest = 0.0f;
					unsigned char * data = motion.data;
					widthStep = motion.cols * motion.elemSize();
					for(int j=0; j<motion.rows; ++j)
					{
						for(int i=0; i<motion.cols; ++i)
						{
							if(data[j*widthStep+i*3+0] ||
									data[j*widthStep+i*3+1] ||
									data[j*widthStep+i*3+2])
							{
								float dist = std::sqrt(float((i-centerROIGlobal.x)*(i-centerROIGlobal.x) + (j-centerROIGlobal.y)*(j-centerROIGlobal.y)));
								if((nearestMovingPixel.x == 0 && nearestMovingPixel.y == 0) ||
									dist < distNearest)
								{
									nearestMovingPixel.y = j;
									nearestMovingPixel.x = i;
									distNearest = dist;
								}
							}
						}
					}
				}

				if(!attentionDisabled && nearestMovingPixel.x && nearestMovingPixel.y)
				{
					if(localAttention)
					{
						ROS_INFO("nearestMovingPixel center(local:global)=(%d,%d:%d,%d) nearest(local:global)=(%d,%d:%d,%d)",
								centerROILocal.x,
								centerROILocal.y,
								centerROILocal.x+roi.x,
								centerROILocal.y+roi.y,
								nearestMovingPixel.x,
								nearestMovingPixel.y,
								nearestMovingPixel.x+roi.x,
								nearestMovingPixel.y+roi.y);
						nearestMovingPixel.x += roi.x;
						nearestMovingPixel.y += roi.y;
						if( nearestMovingPixel.x > roi.x &&
							nearestMovingPixel.x<roi.x+roi.width &&
							nearestMovingPixel.x>roi.width/2 &&
							nearestMovingPixel.x<ptr->image.cols - roi.width/2)
						{
							roi.x = nearestMovingPixel.x-roi.width/2;
						}
						if( nearestMovingPixel.y>roi.y &&
							nearestMovingPixel.y<roi.y+roi.height &&
							nearestMovingPixel.y>roi.height/2 &&
							nearestMovingPixel.y<ptr->image.rows - roi.height/2)
						{
							roi.y = nearestMovingPixel.y-roi.height/2;
						}
					}
					else
					{
						ROS_INFO("nearestMovingPixel center(global)=(%d,%d) nearest(global)=(%d,%d)",
								centerROIGlobal.x,
								centerROIGlobal.y,
								nearestMovingPixel.x,
								nearestMovingPixel.y);
						if(	nearestMovingPixel.x>roi.width/2 &&
							nearestMovingPixel.x<ptr->image.cols - roi.width/2)
						{
							roi.x = nearestMovingPixel.x-roi.width/2;
						}
						else if(nearestMovingPixel.x>=ptr->image.cols - roi.width/2)
						{
							roi.x = ptr->image.cols - roi.width - 1;
						}
						else
						{
							roi.x =0;
						}
						if(	nearestMovingPixel.y>roi.height/2 &&
							nearestMovingPixel.y<ptr->image.rows - roi.height/2)
						{
							roi.y = nearestMovingPixel.y-roi.height/2;
						}
						else if(nearestMovingPixel.y>=ptr->image.rows - roi.height/2)
						{
							roi.y = ptr->image.rows - roi.height - 1;
						}
						else
						{
							roi.y = 0;
						}
					}
				}
				cv::Mat newImageROI = cv::Mat(ptr->image, roi);
				int radius = newImageROI.cols/2;
				float M = POLAR_RINGS/std::log(radius);
				previousPolarROI = cv::Mat(POLAR_RAYS, POLAR_RINGS, CV_8UC3);
				IplImage iplPolar = previousPolarROI;
				IplImage iplImageROI = newImageROI;
				cvLogPolar( &iplImageROI, &iplPolar, cvPoint2D32f(radius, radius), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
			}
			else
			{
				int size = ptr->image.cols < ptr->image.rows?ptr->image.cols/ROI_RATIO: ptr->image.rows/ROI_RATIO;
				int x = size<ptr->image.cols?(ptr->image.cols-size)/2:0;
				int y = size<ptr->image.rows?(ptr->image.rows-size)/2:0;
				roi = cv::Rect(x, y, size, size);
			}
			previousGlobalImage = ptr->image.clone();

			ROS_INFO("polar ROI (%d,%d,%d,%d)", roi.x, roi.y, roi.width, roi.height);
			cv::Mat localImage = cv::Mat(ptr->image, roi).clone();

			/*int radius = polarImage.cols/2;
			float M = 64/std::log(radius);
			ROS_INFO("src size=(%d,%d) radius=%d, M=%f", polarImage.cols, polarImage.rows, radius, M);
			cv::Mat tmpPolar(128, 64, CV_8UC3);
			IplImage iplTmpPolar = tmpPolar;
			IplImage iplPolar = polarImage;
			cvLogPolar( &iplPolar, &iplTmpPolar, cvPoint2D32f(radius, radius), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
			cvLogPolar( &iplTmpPolar, &iplPolar, cvPoint2D32f(radius, radius), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS+CV_WARP_INVERSE_MAP );
*/

			if(rosPublisherLocal.getNumSubscribers())
			{
				cv_bridge::CvImage img;
				img.header.frame_id = msg->header.frame_id;
				img.header.stamp = msg->header.stamp;
				img.encoding = sensor_msgs::image_encodings::BGR8;
				img.image = localImage;
				rosPublisherLocal.publish(img.toImageMsg());
			}


			if(rosPublisherMotionGlobal.getNumSubscribers())
			{
				cv_bridge::CvImage img;
				img.header.frame_id = msg->header.frame_id;
				img.header.stamp = msg->header.stamp;
				cv::rectangle(motion, cv::Point2f(roi.x, roi.y), cv::Point2f(roi.x+roi.width, roi.y+roi.height), cv::Scalar(0, 255, 0), 1);
				img.encoding = sensor_msgs::image_encodings::BGR8;
				img.image = motion;
				rosPublisherMotionGlobal.publish(img.toImageMsg());
			}

			if(rosPublisherWithRoi.getNumSubscribers())
			{
				cv::Mat imageWithRoi = ptr->image.clone();
				cv::rectangle(imageWithRoi, cv::Point2f(roi.x, roi.y), cv::Point2f(roi.x+roi.width, roi.y+roi.height), cv::Scalar(0, 255, 0), 1);
				cv_bridge::CvImage img;
				img.header.frame_id = msg->header.frame_id;
				img.header.stamp = msg->header.stamp;
				img.encoding = sensor_msgs::image_encodings::BGR8;
				img.image = imageWithRoi;
				rosPublisherWithRoi.publish(img.toImageMsg());
			}

		}
	}
}

void my_handler(int s){
	QApplication::closeAllWindows();
	QApplication::exit();
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "visual_attention");

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);
	rosPublisherMotionGlobal = it.advertise("image_motion", 1);
	rosPublisherMotionLocal = it.advertise("image_motion_local", 1);
	rosPublisherLocal = it.advertise("image_local", 1);
	rosPublisherMotionLocalPolar = it.advertise("image_motion_local_polar", 1);
	rosPublisherMotionLocalPolarReconstructed = it.advertise("image_motion_local_polar_reconstructed", 1);
	rosPublisherLocalPolar = it.advertise("image_local_polar", 1);
	rosPublisherLocalPolarReconstructed = it.advertise("image_local_polar_reconstructed", 1);
	rosPublisherWithRoi = it.advertise("image_with_roi", 1);

	bool show_gui = true;
	if(show_gui)
	{
		QApplication app(argc, argv);

		QWidget widget;
		widget.setLayout(new QVBoxLayout());
		spinX = new QSpinBox(&widget);
		spinY = new QSpinBox(&widget);
		polarCheckBox = new QCheckBox("Polar", &widget);
		polarCheckBox->setChecked(false);
		QHBoxLayout * hLayout = new QHBoxLayout();
		hLayout->addWidget(spinX);
		hLayout->addWidget(spinY);
		hLayout->addWidget(polarCheckBox);
		widget.layout()->addItem(hLayout);
		UPlot * plot = new UPlot(&widget);
		plot->setWindowTitle("Pixel value");
		plot->keepAllData(false);
		plot->setMaxVisibleItems(50);
		plot->setXLabel("Time (s)");
		g_curveR = plot->addCurve("R", Qt::red);
		g_curveG = plot->addCurve("G", Qt::green);
		g_curveB = plot->addCurve("B", Qt::blue);
		plot->setMinimumSize(600, 400);
		widget.layout()->addWidget(plot);
		widget.show();

		UPlot plotRatio;
		plotRatio.setWindowTitle("Pixel ratio");
		plotRatio.keepAllData(false);
		plotRatio.setMaxVisibleItems(50);
		plotRatio.setXLabel("Time (s)");
		g_curveRRatio = plotRatio.addCurve("R", Qt::red);
		g_curveGRatio = plotRatio.addCurve("G", Qt::green);
		g_curveBRatio = plotRatio.addCurve("B", Qt::blue);
		plotRatio.setMinimumSize(600, 400);
		plotRatio.show();

		// Catch ctrl-c to close the gui
		// (Place this after QApplication's constructor)
		struct sigaction sigIntHandler;
		sigIntHandler.sa_handler = my_handler;
		sigemptyset(&sigIntHandler.sa_mask);
		sigIntHandler.sa_flags = 0;
		sigaction(SIGINT, &sigIntHandler, NULL);

		ros::AsyncSpinner spinner(4); // Use 4 threads
		spinner.start();
		app.exec();
		spinner.stop();
	}
	else
	{
		ros::spin();
	}

	return 0;
}
