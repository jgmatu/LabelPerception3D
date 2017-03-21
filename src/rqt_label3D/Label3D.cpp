#include <ros/ros.h>

#include "rqt_label3D/Label3D.h"
#include <pluginlib/class_list_macros.h>

#include <dirent.h>
#include <sys/types.h>
#include <string.h>
#include <sstream>

namespace rqt_label3D {

Label3D::Label3D()
: rqt_gui_cpp::Plugin()
, widget_(0)
, rgb_topic_("/camera/rgb/image_raw")
, depth_topic_("/camera/depth/image")
, it_(nh_)
, subRGB_(it_, rgb_topic_, 1)
, subDepth_(it_, depth_topic_, 1)
, sync( MySyncPolicy( 10 ), subRGB_, subDepth_ )
, seq_(0)
, pseq(0)
, demo_(false)
, nseqs(getSeqs())
{
	// Constructor is called first before initPlugin function, needless to say.
	// give QObjects reasonable names.
	setObjectName("Label3D");
	sync.registerCallback( boost::bind( &Label3D::callbackImages, this, _1, _2));
	setFirstElement();
}


/*
 * Crear la UI con los botones, cajas y ventanas de imagen
 * que va a tener la UI.
 */
void
Label3D::initPlugin(qt_gui_cpp::PluginContext& context) {
	// access standalone command line arguments.
	QStringList argv = context.argv();

	// create QWidget.
	widget_ = new QWidget();

	// extend the widget with all attributes and children from UI file.
	ui_.setupUi(widget_);
	ui_.layerBox->setMaximum(LAYERS - 1);
	ui_.layerBox->setMinimum(0);
	ui_.categoryBox->setMaximum(CATEGORIES);
	ui_.categoryBox->setMinimum(0);

	connect(ui_.layerBox 		 , SIGNAL(valueChanged(int)) , this , SLOT(layerChanged(int)));
	connect(ui_.categoryBox 	 , SIGNAL(valueChanged(int)) , this , SLOT(categoryChanged(int)));
	connect(ui_.pixelModeButton  , SIGNAL(clicked(bool)) 	 , this , SLOT(setPixelMode(bool)));
	connect(ui_.deleteModeButton , SIGNAL(clicked(bool)) 	 , this , SLOT(setDeleteMode(bool)));
	connect(ui_.markerSize 		 , SIGNAL(valueChanged(int)) , this , SLOT(markerSizeChanged(int)));
	connect(ui_.generateButton 	 , SIGNAL(clicked(bool)) 	 , this , SLOT(generateFile(bool)));
	connect(ui_.rewardButton     , SIGNAL(clicked(bool))	 , this , SLOT(rewind(bool)));
	connect(ui_.forwardButton    , SIGNAL(clicked(bool))	 , this , SLOT(forward(bool)));

	context.addWidget(widget_);
}

void
Label3D::shutdownPlugin() {
	// TODO unregister all publishers here
}

/*
 *	cv::Vec3b hsv = images_[HSV].at<cv::Vec3b>(j , i);
 *	data[i][j][0] = static_cast<int>(hsv[0]) / 255.0;
 *	data[i][j][1] = static_cast<int>(hsv[1]) / 255.0;
 *	data[i][j][2] = static_cast<int>(hsv[2]) / 180.0;
 */

/*
void
Label3D::rewind (bool activate) {
	ROS_WARN("REWIND READING FILE...");

	std::ifstream file("/home/javi/tfg/dataseets/alextoma1/datain_9052.npy");
	std::string str;
	std::list<unsigned char> pixels;

	int pos = 0;
	while (std::getline(file, str)) {
		QString winOpacity(str.c_str());
		unsigned char pixel = winOpacity.toDouble() * 255;
		pixels.push_front(pixel);
		pos++;
	}
	file.close();

	cv::Mat image(IMG_W, IMG_H, CV_8UC3, cv::Scalar(110 , 150 , 150));
	int channels = image.channels() , step = image.step;
	for (int i = 0 ; i < IMG_W; i++) {
		for (int j = 0 ; j < IMG_H; j++) {
			int posdata = j * step + i * channels;
			for (int k = 0; k < channels ; k++) {
				image.data[posdata + k] = pixels.back();
				pixels.pop_back();
			}
		}
	}
	ui_.imageView->setImage(image, HSV, ros::Time());
}
*/

void
readFileIn(std::string filename, float datain_[IMG_W][IMG_H][6]) {
	std::ifstream file(filename.c_str());
	std::string str;

	for (int i = 0 ; i < IMG_W; i++) {
		for (int j = 0 ; j < IMG_H; j++) {
			for (int k = 0 ; k < 6; k++){
				if (std::getline(file, str)) {
					QString winOpacity(str.c_str());
					datain_[i][j][k] = winOpacity.toDouble();
				}
			}
		}
	}
	file.close();
}

/*
cv::Mat
rotateImage(cv::Mat image , double angle) {
	cv::Mat rotate;
	cv::Point2f pt(image.cols / 2., image.rows/2.);
	cv::Mat aux = cv::getRotationMatrix2D(pt , angle, 1.0);
	cv::warpAffine(image, rotate , aux, cv::Size(image.cols , image.rows));
	return rotate;
}

cv::Mat
rotateImg(cv::Mat image) {
	cv::Mat result(IMG_H, IMG_W, CV_8UC3, cv::Scalar(110 , 260 , 100));

	for (int i = 0 ; i < IMG_H ; i++){
		for (int j = 0 ; j < IMG_W ; j++) {
			cv::Vec3b hsv = image.at<cv::Vec3b>(j , i);
		}
	}
	return result;
}
*/

void
initDataIn(float datain_[IMG_W][IMG_H][6]) {
	for (int i = 0; i < IMG_W; i++) {
		for (int j = 0; j < IMG_H; j++) {
			for (int k = 0 ; k < 6; k++) {
				datain_[i][j][k] = 0.0;
			}
		}
	}
}

void
Label3D::nextImage(std::string filename) {
	if (filename != "") {
		float datain_[IMG_W][IMG_H][6];
		initDataIn(datain_);

		readFileIn(filename, datain_);
		ui_.imageView->setSourceData(datain_);
	}
}

bool
isName(std::string name, std::string prefix) {
	return name.find(prefix) != name.npos;
}

bool
isFileIn(std::string fname , std::string seq) {
	return fname.find(seq) != fname.npos && isName(fname , "datain_");
}

bool isFileOut(std::string fname, std::string seq){
	return fname.find(seq) != fname.npos && isName(fname , "dataout_");
}

std::vector<int>
Label3D::getSeqs() {
	DIR *dr = NULL;
	struct dirent *drnt = NULL;
	std::vector<int> seqs(MAX_FRAMES);

	if ((dr = opendir(".")) == NULL) {
		return seqs;
	}
	int pos = 0;
	while ((drnt = readdir(dr)) != NULL) {
		std::string fname(drnt->d_name);
		if (isName(fname, "datain_")) {
			std::string nseq(fname);
			nseq.erase(0 , 7);
			nseq.erase(4 , fname.length());
			seqs.at(pos) = atoi(nseq.c_str());;
			pos++;
		}
	}
	closedir(dr);
	std::sort(seqs.begin(), seqs.end());
	return seqs;
}

void
Label3D::setFirstElement() {
	for (int i = 0; i < nseqs.size() ; i++) {
		if (nseqs.at(i) == 0) {
			pseq++;
		}
	}
}

std::string
Label3D::setSeqPrefix(int nseq) {
	std::ostringstream seq;

	seq << (nseqs.at(nseq));

	std::string seq_prefix = seq.str();

	seq_prefix.insert(0 , 1 ,'_');
	seq_prefix.insert(seq_prefix.length() , 1, '.');

	return seq_prefix;
}

std::string
Label3D::getFileName(int nseq , bool isIn) {
	DIR *dr = NULL;
	struct dirent *drnt = NULL;
	std::string result("");

	if ((dr = opendir(".")) == NULL) {
		return result;
	}
	while ((drnt = readdir(dr)) != NULL) {
		std::string fname(drnt->d_name);
		if (isFileIn(fname, setSeqPrefix(nseq)) && isIn) {
			result = fname;
			break;
		}
		if (isFileOut(fname , setSeqPrefix(nseq)) && !isIn) {
			result = fname;
			break;
		}
	}
	return result;
}

/*
void
printData(float data[IMG_W][IMG_H][CATEGORIES]) {
	for (int i = 0 ; i < IMG_W; i++) {
		for (int j = 0 ; j < IMG_H; j++) {
			for (int k = 0; k < CATEGORIES; k++) {
				ROS_WARN("%f", data[i][j][k]);
			}
		}
	}
}
*/

/*
void
printLabels(std::list<float> labels) {
	for (std::list<float>::const_iterator it = labels.begin(); it != labels.end(); it++) {
		if (*it > 0.0) {
			ROS_WARN("%f" , *it);
		}
	}
}
*/

void
initDataLabels(float data[IMG_W][IMG_H][CATEGORIES]) {
	for (int i = 0; i < IMG_W; i++) {
		for (int j = 0 ; j < IMG_H; j++) {
			for (int k = 0 ; k < CATEGORIES; k++) {
				data[i][j][k] = 0.0;
			}
		}
	}
}

void
Label3D::setLabels(std::string filename) {
	std::ifstream file(filename.c_str());
	std::string str;
	float dataout_[IMG_W][IMG_H][CATEGORIES];
	initDataLabels(dataout_);

	for (int i = 0 ; i < IMG_W; i++) {
		for (int j = 0 ; j < IMG_H; j++) {
			for (int k = 0 ; k < CATEGORIES ; k++) {
				if(std::getline(file , str)) {
					QString winOpacity(str.c_str());
					dataout_[i][j][k] = winOpacity.toDouble();
				}
			}
		}
	}
	file.close();
	ui_.imageView->setResultData(dataout_);
}

void
Label3D::setImage() {
	bool isIn = true;
	demo_ = true;
	if (nseqs.at(pseq) != 0) {
		setLabels(getFileName(pseq , !isIn));
		nextImage(getFileName(pseq , isIn));
		seq_ = nseqs.at(pseq);
	}
}

void
Label3D::rewind (bool activate) {
	pseq = (pseq - 1) % nseqs.size();
	setImage();
}

void
Label3D::forward (bool activate) {
	pseq = (pseq + 1) % nseqs.size();
	setImage();
}

void
Label3D::layerChanged(int newVal) {
	if (newVal != LAYERS) {
		ui_.imageView->changeLayer(newVal);
	} else {
		ROS_WARN("BAD CATEGORY IMAGE LAYER");
	}
}

void
Label3D::generateFile(bool activate) {

	ui_.imageView->getSourceData(in_data);
	ui_.imageView->getResultData(out_data);

	std::stringstream fin_ss;
	std::stringstream fout_ss;

	fin_ss << "datain_" << seq_ << ".npy";
	fout_ss << "dataout_" << seq_ << ".npy";

	// Write data_in file data....
	std::ofstream in_file (fin_ss.str().c_str());
	if (in_file.is_open()) {
		for(int i = 0; i < IMG_W; i++) {
			for(int j = 0; j < IMG_H; j++) {
				for(int k = 0; k < 6; k++) {
					// H, S, V, Depth, DD, DC...
					in_file << in_data[i][j][k] << "\n";
				}
			}
		}
	}
	in_file.close();

	// Write data_out file data...
	std::ofstream out_file (fout_ss.str().c_str());
	if (out_file.is_open()) {
		for(int i = 0; i < IMG_W; i++) {
			for(int j = 0; j < IMG_H; j++) {
				for(int k = 0; k < CATEGORIES; k++) {
					out_file << out_data[i][j][k] << "\n";
				}
			}
		}
	}
	out_file.close();
}

void
Label3D::markerSizeChanged(int newVal) {
	ui_.imageView->changeMarkerSize(newVal);
}

void
Label3D::categoryChanged(int newVal) {
	if (newVal != CATEGORIES) {
		ui_.imageView->changeCategory(newVal);
	} else {
		ROS_WARN("BAD CATEGORY Image");
	}
}

void
Label3D::setPixelMode(bool activate) {
	ui_.imageView->changeMode(PIXEL);
}

void
Label3D::setDeleteMode(bool activate) {
	ui_.imageView->changeMode(DELETE);
}

void
Label3D::callbackImages(const sensor_msgs::Image::ConstPtr& rgbMsg, const sensor_msgs::Image::ConstPtr& depthMsg) {

	seq_ = rgbMsg->header.seq;
	try {

		cv_bridge::CvImageConstPtr cv_rgb_ptr = cv_bridge::toCvShare(rgbMsg, sensor_msgs::image_encodings::RGB8);
		cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depthMsg,  sensor_msgs::image_encodings::TYPE_32FC1);

		cv::Mat hsvImage;
		cv::cvtColor(cv_rgb_ptr->image, hsvImage,  CV_RGB2HSV);

		ui_.imageView->setImage(hsvImage, HSV, cv_rgb_ptr->header.stamp);
		ui_.imageView->setImage(cv_depth_ptr->image, DEPTH, cv_depth_ptr->header.stamp);
		ui_.imageView->updateImage();

	} catch (cv_bridge::Exception& e) {
		ROS_INFO("RGB Image Exception Received");
	}
}

} // namespace...
PLUGINLIB_DECLARE_CLASS(rqt_label3D, Label3D, rqt_label3D::Label3D, rqt_gui_cpp::Plugin)
