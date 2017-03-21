#ifndef rqt_label3D_plugin_H
#define rqt_label3D_plugin_H

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <rqt_gui_cpp/plugin.h>
#include <ui_label3D.h>
#include <QWidget>
#include <QMenu>
#include <QInputDialog>
#include <QLabel>

#include <QMessageBox>

#include <list>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include "rqt_label3D/CustomLabel.h"


namespace rqt_label3D {

typedef image_transport::SubscriberFilter ImageSubscriber;
typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

class Label3D : public rqt_gui_cpp::Plugin
{
	Q_OBJECT

public :

  	Label3D();
  	virtual void initPlugin (qt_gui_cpp::PluginContext& context);
  	virtual void shutdownPlugin ();

protected :

  	virtual void callbackImages(const sensor_msgs::Image::ConstPtr& rgbMsg,
				const sensor_msgs::Image::ConstPtr& depthMsg);

	std::string rgb_topic_;
	std::string depth_topic_;

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;

	ImageSubscriber subRGB_;
	ImageSubscriber subDepth_;
	message_filters::Synchronizer < MySyncPolicy > sync;

public slots :

	void layerChanged (int newVal);
	void categoryChanged (int newVal);
	void markerSizeChanged (int newVal);

	void setPixelMode (bool activate);
	void setDeleteMode (bool activate);
	void generateFile (bool activate);
	void rewind (bool activate);
	void forward (bool activate);

private :

	static const int MAX_FRAMES = 2000;

	Ui::Label3DWidget ui_;
	QWidget* widget_;

	float in_data[IMG_W][IMG_H][6]; // HSV, DEPTH, DELTADEPTH, DELTAHSV. Why 6?
	float out_data[IMG_W][IMG_H][CATEGORIES]; // JAVI, ALEX, NOCATEGORY.
	bool demo_;

	int seq_;
	std::vector<int> nseqs;
	int pseq;

	std::string getFileName(int nseq , bool isIn);
	std::vector<int> getSeqs();

	void setImage();
	void setLabels(std::string filename);

	void setFirstElement();
	std::string setSeqPrefix(int nseq);

	void nextImage(std::string filename);
	cv::Mat getImage(std::list<unsigned char> pixels);
};

}  		// namespace rqt_vicode
#endif  // rqt_label3D_plugin_H
