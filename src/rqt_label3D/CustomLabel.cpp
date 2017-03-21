#include "rqt_label3D/CustomLabel.h"

namespace rqt_label3D {

const float CustomLabel::MAX_DEPTH_DIST = 10.f;
const float CustomLabel::MIN_DEPTH_DIST = 0.1f;
const float CustomLabel::MAX_DEPTH_DELTA = 20.f;
const float CustomLabel::MIN_DEPTH_DELTA = -20.0f;

std::string
type2str(int type) {
  std::string r;
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U  :  r = "8U"; break;
    case CV_8S  :  r = "8S"; break;
    case CV_16U :  r = "16U"; break;
    case CV_16S :  r = "16S"; break;
    case CV_32S :  r = "32S"; break;
    case CV_32F :  r = "32F"; break;
    case CV_64F :  r = "64F"; break;
    default:       r = "User"; break;
  }

  r += "C";
  r += (chans + '0');
  return r;
}


/*
 * Constructor de la clase crea en memoria una
 * nueva matriz con la resolucion de la imagen 640*480*CATEGORIAS
 * de tipo float que se rellenara con los pixeles con su correspondiente
 * categoria.
 */
CustomLabel::CustomLabel(QWidget* parent, Qt::WindowFlags flags) :
	QLabel(parent),
	currentLayer_(HSV),
	currentCategory_(JAVI),
	currentMode_(PIXEL),
	markerSize_(1),
	started_(false),
	hasHSVPairs_(false),
	hasDepthPairs_(false)
{
	memset(labels_, 0, IMG_W * IMG_H * CATEGORIES * sizeof(float));

	 for(int i = 0; i < IMG_W; i++) {
	 	for(int j = 0; j < IMG_H; j++) {
	 		for(int k = 0; k < CATEGORIES; k++) {
	 			labels_[i][j][k] = 0;
	 		}
	 	}
	 }
}

/*
 * Evento, si el raton es pulsado o movido y se esta pulsando el
 * boton izquierdo se mira el punto en el que esta el raton y se realizara
 * el filtro en la imagen segun esté en pixel, con su categoria o delete
 * para eliminar la categoria en la imagen.
 */
bool
CustomLabel::event(QEvent *myEvent) {
	if (myEvent->type() == QEvent::MouseMove || myEvent->type() == QEvent::MouseButtonPress) {
		const QMouseEvent* const me = static_cast<const QMouseEvent*>( myEvent );

		if (me->buttons() == Qt::LeftButton) {
			const QPoint p = me->pos();

			switch(currentMode_) {
				case PIXEL:
					setFilter(p.x(), p.y(), currentCategory_);
					break;
				case DELETE:
					setFilter(p.x(), p.y(), NOCATEGORY);
					break;
			}
			updateImage();
		}
	}
	return QWidget::event(myEvent);
}

void
CustomLabel::slotLabelClicked() {
	// Implementation of Slot which will consume signal
	;
}

void
CustomLabel::setDeltaFilter(const cv::Mat& last, const cv::Mat& current) {
//	images_[DELTA_DEPTH] = current.clone();
//	images_[DELTA_DEPTH] -= last;
    // accept only char type matrices
	images_[DELTA_DEPTH] = current.clone();

//	CV_Assert(images_[DELTA_DEPTH].depth() == CV_8UC1);

	for (int i = 0 ; i < IMG_W ; i++) {
		for (int j = 0 ; j < IMG_H * current.channels() ; j++) {
			unsigned char delta =  images_[DELTA_DEPTH].at<uchar>(i , j) - last.at<uchar>(i , j);
			if (delta > 50) {
				images_[DELTA_DEPTH].at<uchar>(i , j) -= last.at<uchar>(i , j);
			} else {
				images_[DELTA_DEPTH].at<uchar>(i , j) = 0;
			}
 		}
	}
}

/*
 * Crea la imagen de diferencia de profundidad, dada la imagen actual la anterior
 * y el tiempo transcurrido entre ambas.
 */
void
CustomLabel::setDeltaDepth(const cv::Mat& last, const cv::Mat& current, ros::Duration elapsed) {
	setDeltaFilter(last , current);

//	images_[DELTA_DEPTH] = current.clone();
//	images_[DELTA_DEPTH] -= last;
//	images_[DELTA_DEPTH] /= elapsed.toSec();

//	images_[DELTA_DEPTH] -= MIN_DEPTH_DELTA;
//	images_[DELTA_DEPTH] /= (MAX_DEPTH_DELTA - MIN_DEPTH_DELTA);
}

/*
 * Crea la imagen diferencia de color , dadas la imagen anterior la actual
 * y el tiempo transcurrido entre ambas.
 */
void
CustomLabel::setDeltaColor(const cv::Mat& last, const cv::Mat& current, ros::Duration elapsed) {
	cv::Mat last_f, current_f;
	current.convertTo(current_f, CV_32F);
	last.convertTo(last_f, CV_32F);

	images_[DELTA_COLOR]  = current_f.clone();
	images_[DELTA_COLOR] -= last_f;
	images_[DELTA_COLOR] /= elapsed.toSec();

	images_[DELTA_COLOR] += 255.0;
	images_[DELTA_COLOR] /= 512.0;
}

/*
 * Pone la imagen en funcion de la capa actual que se
 * tenga seleccionada, DEPTH o HSV, y se hace su correspondiente
 * diferencia.
 */
void
CustomLabel::setImage(cv::Mat image, int layer, ros::Time stamp) {

	if (layer == DEPTH) {

		if(stamp.toSec() > 0.00001) {
			elapsedDepth_ = stamp - lastDepthTS_;
			lastDepthTS_ = stamp;
		} else {
			elapsedDepth_.fromSec(1.0);
		}
		if (hasDepthPairs_) {
			setDeltaDepth(images_[layer], image, elapsedDepth_);
		}
		hasDepthPairs_ = true;

	} else if (layer == HSV) {

		if(stamp.toSec() > 0.00001) {
			elapsedColor_ = stamp - lastColorTS_;
			lastColorTS_ = stamp;
		} else {
			elapsedColor_.fromSec(1.0);
		}
		if (hasHSVPairs_) {
			// Por qué, tengo un array de 3 objetos cv::Mat
			cv::Mat current_layers[3];
			cv::Mat last_layers[3];

			// void split(InputArray m, OutputArrayOfArrays mv);
			cv::split(image, current_layers);
			cv::split(images_[layer], last_layers);
			setDeltaColor(last_layers[2], current_layers[2], elapsedColor_);
		}
		hasHSVPairs_ = true;
	}
	started_ = true;
	images_[layer] = image.clone(); // Imagenes de capas, HSV, DEPTH , deltaHSV, deltaDepth,
									// se copia la imagen en la capa correspondiente...
	updateImage();
}

/*
 * Cambia la capa actual de imagen con la que se
 * esté trabajando. HSV, DEPTH, DELTAHSV, DELTADEPTH.
 */
void
CustomLabel::changeLayer(int layer) {
	if(layer != currentLayer_) {
		currentLayer_ = layer;
		updateImage();
	}
}

/*
 * Recorre todos los pixeles del marco que tenga
 * de la mascara que pone los pixeles a una determinada
 * categoria, rellena la matriz de imagenxcategoria con la
 * categoria seleccionada dentro del marco o mascara de filtro
 * de la imagen para poner la categoria.
 */
void
CustomLabel::setFilter(int pi, int pj, int category) {
	for(int i = -markerSize_; i < markerSize_; i++) {
		for(int j = -markerSize_; j < markerSize_; j++) {
			int cx = pi + i;
			int cy = pj + j;

			if(cx >= 0 && cx < IMG_W && cy >= 0 && cy < IMG_H) {
				for(int k = 0; k < CATEGORIES; k++) {
					if(k == category) {
						labels_[pi+i][pj+j][k] = 1.0;
					} else {
						labels_[pi+i][pj+j][k] = 0.0;
					}
				}
			}
		}
	}
}

/*
 * Obtengo los pixeles que tienen una determinada categoria,
 * y en función de la categoria a los que pertenezca ese pixel
 * de la imagen en la matriz de etiquetas se colorera en azul,
 * verde o no se colorea el pixel.
 */
QImage
CustomLabel::getImageFilter() {
	QImage image(IMG_W, IMG_H, QImage::Format_ARGB32);

	for(int i = 0; i < image.width(); i++) {
		for(int j = 0; j < image.height(); j++) {
			image.setPixel(i, j, QColor(0, 0, 0, 0).rgba());
		}
	}
	for(int i = 0; i < IMG_W; i++) {
		for(int j = 0; j < IMG_H; j++) {
			if(labels_[i][j][JAVI] > 0.1) {
				image.setPixel(i, j, QColor(0, 0, 255, 100).rgba());
			} else if(labels_[i][j][ALEX] > 0.1) {
				image.setPixel(i, j, QColor(0, 255, 0, 100).rgba());
			}
			//	else if(labels_[i][j][CHAIR]>0.1) image.setPixel(i, j, QColor(0, 0, 255, 100).rgba());
			//	else if(labels_[i][j][BED]>0.1) image.setPixel(i, j, QColor(255, 0, 255, 100).rgba());
		}
	}
	return image;
}

/*
 * Actualiza la imagen en la pantalla dependiendo
 * de la capa que tengamos selecionada en la UI.
 *
 * Si la imagen es la capa HSV la conversion a RGB es directa.
 *
 * Si la capa es depth, se convierte a tonos de gris realizando una
 * normalizacion de la profundidad entre 0..255
 *
 * Si la imagen es deltadepth hago converto a imagen escalada en unsigned char.
 * para pintar la imagen en pantalla en tonos de gris...
 *
 * Si es delta de color, lo mismo conviert la imagen escalada a uchar para pintar
 * la imagen en tonos de gris de 0...255
 */
void
CustomLabel::updateImage() {
	QImage img_filter = getImageFilter();
	cv::Mat aux;

	if (currentLayer_ == HSV) {

		cv::Mat hsvrgb;

		cv::cvtColor(images_[HSV], hsvrgb, CV_HSV2RGB);
		aux = hsvrgb;

	} else if (currentLayer_ == DEPTH) {

		cv::Mat img_scaled_8u, rgbdepth;
		// void Mat::convertTo(OutputArray m, int rtype, double alpha=1, double beta=0) const. (640x480).
		images_[DEPTH].convertTo(img_scaled_8u, CV_8UC1, 255. / (MAX_DEPTH_DIST - MIN_DEPTH_DIST));
		applyColorMap(img_scaled_8u, rgbdepth, cv::COLORMAP_BONE);

		aux = rgbdepth;

	} else if (currentLayer_ == DELTA_DEPTH) {

		cv::Mat img_scaled_8u, deltadepth;
		// void Mat::convertTo(OutputArray m, int rtype, double alpha=1, double beta=0) const. (640x480).
		images_[DELTA_DEPTH].convertTo(img_scaled_8u, CV_8UC1, 255.);
		applyColorMap(img_scaled_8u, deltadepth, cv::COLORMAP_PARULA);

		aux = deltadepth;

	} else if (currentLayer_ == DELTA_COLOR) {

		cv::Mat img_scaled_8u, deltacolor;
		// void Mat::convertTo(OutputArray m, int rtype, double alpha=1, double beta=0) const. (640x480).
		images_[DELTA_COLOR].convertTo(img_scaled_8u, CV_8UC1, 255.);
		applyColorMap(img_scaled_8u, deltacolor, cv::COLORMAP_BONE);

		aux = deltacolor;

	} else {

		fprintf(stderr, "Not valid layer\n");

	}

	QImage img = QImage(aux.data, aux.cols, aux.rows, aux.step[0], QImage::Format_RGB888);

	QPixmap pixmap(IMG_W, IMG_H);
	QPainter p(&pixmap);

	p.drawImage(0, 0, img, 0, 0, -1, -1, Qt::DiffuseAlphaDither);
	p.drawImage(0, 0, img_filter, 0, 0, -1, -1, Qt::DiffuseAlphaDither);
	setPixmap(pixmap);
}

void
CustomLabel::changeMode(int mode) {
	currentMode_ = mode;
}

/*
 * Cambia la categoria, en nuestro caso ALEX, JAVI, NO CATEGORY;
 */
void
CustomLabel::changeCategory(int category) {
	if (category != CATEGORIES) {
		currentCategory_ = category;
	}
}

void
CustomLabel::changeMarkerSize(int size) {
	markerSize_ = size;
}

void
CustomLabel::getSourceData(float data[IMG_W][IMG_H][6]) {
	if (!started_) {
		return;
	}

	for (int i = 0; i < IMG_W; i++) {
	 	for (int j = 0; j < IMG_H; j++) {
	 		// HSV...
			cv::Vec3b hsv = images_[HSV].at<cv::Vec3b>(j , i);

			data[i][j][0] = static_cast<int>(hsv[0]) / 255.0;
			data[i][j][1] = static_cast<int>(hsv[1]) / 255.0;
			data[i][j][2] = static_cast<int>(hsv[2]) / 180.0;

			// Depth...
			float d = images_[DEPTH].at<float>(j, i);

			if (std::isinf(d) || std::isnan(d)) {
				data[i][j][3] =  1.0;
			} else {
				data[i][j][3] = (d - MIN_DEPTH_DIST) / (MAX_DEPTH_DIST- MIN_DEPTH_DIST);
			}
			data[i][j][3] = std::min(std::max(data[i][j][3], -1.0f), 1.0f);

			// Delta Depth...
			float dd = images_[DELTA_DEPTH].at<float>(j, i);

			if (std::isinf(dd) || std::isnan(dd)) {
				data[i][j][4] =  1.0;
			} else {
				data[i][j][4] = dd;
			}
			data[i][j][4] = std::min(std::max(data[i][j][4], -1.0f), 1.0f);

			// Delta Color...
			float dc = images_[DELTA_COLOR].at<float>(j, i);

			if (std::isinf(dc) || std::isnan(dc)) {
				data[i][j][5] =  1.0;
			} else {
				data[i][j][5] = dc;
			}
			data[i][j][5] = std::min(std::max(data[i][j][4], -1.0f), 1.0f);
		}
	}
}

void
CustomLabel::getImageHSV(float datain_[IMG_W][IMG_H][6]) {
	cv::Mat hsv(IMG_H, IMG_W, CV_8UC3, cv::Scalar(110 , 150 , 150));
	int channels = hsv.channels() , step = hsv.step;

	for (int i = 0 ; i < IMG_W; i++) {
		for (int j = 0 ; j < IMG_H; j++) {
			int posdata = j * step + i * channels;
			for (int k = 0; k < 3; k++) {
				if (k < 2) {
					hsv.data[posdata + k] = datain_[i][j][k] * 255.0;
				}
				hsv.data[posdata + 2] = datain_[i][j][k] * 180.0;
			}
		}
	}
	images_[HSV] = hsv;
}

void
CustomLabel::getImageDepth(float datain_[IMG_W][IMG_H][6]) {
	cv::Mat image(IMG_H, IMG_W, CV_8UC1, cv::Scalar(100));
	int channels = image.channels() , step = image.step;

	for (int i = 0 ; i < IMG_W; i++) {
		for (int j = 0; j < IMG_H; j++) {
			int posdata = j * step + i * channels;
			if (datain_[i][j][3] >= 1.0) {
				image.data[posdata] = std::numeric_limits<float>::quiet_NaN();
			} else {
				image.data[posdata] = datain_[i][j][3] * (MAX_DEPTH_DIST - MIN_DEPTH_DIST) + MIN_DEPTH_DIST;
			}
		}
	}
	images_[DEPTH] = image;
}

void
CustomLabel::getImageDeltaDepth(float datain_[IMG_W][IMG_H][6]) {
	cv::Mat delta_depth(IMG_H, IMG_W, CV_8UC1, cv::Scalar(100));
	int channels = delta_depth.channels() , step = delta_depth.step;

	for (int i = 0 ; i < IMG_W; i++) {
		for (int j = 0; j < IMG_H; j++) {
			int posdata = j * step + i * channels;
			if (datain_[i][j][4] >= 1.0) {
				delta_depth.data[posdata] = std::numeric_limits<float>::quiet_NaN();
			} else {
				delta_depth.data[posdata] = datain_[i][j][4] * 255.0;
			}
		}
	}
	images_[DELTA_DEPTH] = delta_depth;
}

void
CustomLabel::getImageDeltaColor(float datain_[IMG_W][IMG_H][6]) {
	cv::Mat delta_color(IMG_H, IMG_W, CV_8UC1, cv::Scalar(100));
	int channels = delta_color.channels() , step = delta_color.step;

	for (int i = 0 ; i < IMG_W; i++) {
		for (int j = 0; j < IMG_H; j++) {
			int posdata = j * step + i * channels;
			if (datain_[i][j][5] >= 1.0) {
				delta_color.data[posdata] = std::numeric_limits<float>::quiet_NaN();
			} else {
				delta_color.data[posdata] = (datain_[i][j][5] * 512.0 - 255.0) * 255.0;
				delta_color.data[posdata + 1] = (datain_[i][j][5] * 512.0 - 255.0) * 255.0;
				delta_color.data[posdata + 2] = (datain_[i][j][5] * 512.0 - 255.0) * 180;
			}
		}
	}
	images_[DELTA_COLOR] = delta_color;
}

void
CustomLabel::setSourceData(float datain_[IMG_W][IMG_H][6]) {
	getImageHSV(datain_);
	getImageDepth(datain_);
	getImageDeltaDepth(datain_);
	getImageDeltaColor(datain_);
	updateImage();
}

void
CustomLabel::getResultData(float data[IMG_W][IMG_H][CATEGORIES]) {
	for(int i = 0; i < IMG_W; i++) {
		for(int j = 0; j < IMG_H; j++) {
			for(int k = 0; k < CATEGORIES; k++) {
				data[i][j][k] = labels_[i][j][k];
			}
	 	}
	}
}

void
CustomLabel::setResultData(float data[IMG_W][IMG_H][CATEGORIES]) {
	for(int i = 0; i < IMG_W; i++) {
		for(int j = 0; j < IMG_H; j++) {
			for(int k = 0; k < CATEGORIES; k++) {
				labels_[i][j][k] = data[i][j][k];
			}
	 	}
	}
}


}
