#ifndef CUSTOMLABEL_H_
#define CUSTOMLABEL_H_

#include <QWidget>
#include <QLabel>
#include <QEvent>
#include <QColor>
#include <QPainter>
#include <QMouseEvent>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>

namespace rqt_label3D {

typedef enum {JAVI, ALEX, CATEGORIES, NOCATEGORY} ImageCategories;
typedef enum {HSV, DEPTH, DELTA_DEPTH, DELTA_COLOR, LAYERS} ImageLayers;
typedef enum {PIXEL, DELETE, MODES} Modes;
typedef enum {IMG_W=640, IMG_H=480} Sizes;

class CustomLabel : public QLabel {
	Q_OBJECT

signals:

    	void myLabelClicked();
		void mouseLeft(int x, int y);

public slots:

		void slotLabelClicked();    // Slot which will consume signal.

public:
		static const float MAX_DEPTH_DIST;
		static const float MIN_DEPTH_DIST;
		static const float MAX_DEPTH_DELTA;
		static const float MIN_DEPTH_DELTA;

		CustomLabel(QWidget* parent, Qt::WindowFlags flags = 0);

		void changeLayer(int);
		void changeCategory(int);
		void changeMode(int mode);
		void changeMarkerSize(int size);

		void setImage(cv::Mat image, int layer, ros::Time stamp);
		void updateImage();

		int getWidth() {return IMG_W;};
		int getHeight() {return IMG_H;};
		int getLayers() {return LAYERS;};
		int getCategories() {return CATEGORIES;};

		void getSourceData(float data[IMG_W][IMG_H][6]); // Where I am indexing?? H S V D DD DC.
		void getResultData(float data[IMG_W][IMG_H][CATEGORIES]);

		void setResultData(float data[IMG_W][IMG_H][CATEGORIES]);
		void setSourceData(float data[IMG_W][IMG_H][6]);

protected:

		bool event(QEvent *myEvent);

private:

		QImage getImageFilter();

		void setFilter(int pi, int pj, int category);
		void setDeltaFilter(const cv::Mat& last, const cv::Mat& current);
		void setDeltaDepth(const cv::Mat& last, const cv::Mat& current, ros::Duration elapsed);
		void setDeltaColor(const cv::Mat& last, const cv::Mat& current, ros::Duration elapsed);

		void getImageHSV(float datain_[IMG_W][IMG_H][6]);
		void getImageDepth(float dataout_[IMG_W][IMG_H][6]);

		void getImageDeltaDepth(float datain_[IMG_W][IMG_H][6]);
		void getImageDeltaColor(float datain_[IMG_W][IMG_H][6]);


		cv::Mat images_[LAYERS];
		char tag[255];

		float labels_[IMG_W][IMG_H][CATEGORIES];

		int currentLayer_;
		int currentCategory_;
		int currentMode_;
		int markerSize_;

		bool started_;
		bool hasHSVPairs_;
		bool hasDepthPairs_;

		ros::Time lastColorTS_;
		ros::Time lastDepthTS_;
		ros::Duration elapsedColor_;
		ros::Duration elapsedDepth_;
};

}
#endif
