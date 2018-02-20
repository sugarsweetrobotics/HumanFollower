// -*- C++ -*-
/*!
 * @file  HumanFollower.cpp
 * @brief Human Follow Commander for Mobile Robots
 * @date $Date$
 *
 * @author Yuki Suga, Sugar Sweet Robotics
 * @ysuga
 *
 * GPLv3
 *
 * $Id$
 */

#include <stdint.h>
#include "HumanFollower.h"

// Module specification
// <rtc-template block="module_spec">
static const char* humanfollower_spec[] =
{
	"implementation_id", "HumanFollower",
	"type_name", "HumanFollower",
	"description", "Human Follow Commander for Mobile Robots",
	"version", "1.0.0",
	"vendor", "SUGAR SWEET ROBOTICS",
	"category", "MobileRobot",
	"activity_type", "PERIODIC",
	"kind", "DataFlowComponent",
	"max_instance", "1",
	"language", "C++",
	"lang_type", "compile",
	// Configuration variables
	"conf.default.debug", "0",
	"conf.default.vx_gain", "1.0",
	"conf.default.va_gain", "1.0",
	"conf.default.offset_human", "0.5",
	"conf.default.forget_time", "1.0",
	// Widget
	"conf.__widget__.debug", "text",
	"conf.__widget__.vx_gain", "text",
	"conf.__widget__.va_gain", "text",
	// Constraints
	""
};
// </rtc-template>


IplImage *pImages[2];
uint32_t imageCount = 0;
bool endFlag = false;

const uint32_t WIDTH = 640;
const uint32_t HEIGHT = WIDTH;


/*!
 * @brief constructor
 * @param manager Maneger Object
 */
HumanFollower::HumanFollower(RTC::Manager* manager)
// <rtc-template block="initializer">
: RTC::DataFlowComponentBase(manager),
m_rangeIn("range", m_range),
m_velocityOut("velocity", m_velocity)

// </rtc-template>
{
}

/*!
 * @brief destructor
 */
HumanFollower::~HumanFollower()
{
}



RTC::ReturnCode_t HumanFollower::onInitialize()
{
	// Registration: InPort/OutPort/Service
	// <rtc-template block="registration">
	// Set InPort buffers
	addInPort("range", m_rangeIn);

	// Set OutPort buffer
	addOutPort("velocity", m_velocityOut);

	// Set service provider to Ports

	// Set service consumers to Ports

	// Set CORBA Service Ports

	// </rtc-template>

	// <rtc-template block="bind_config">
	// Bind variables and configuration variable
	bindParameter("debug", m_debug, "0");
	bindParameter("vx_gain", m_vx_gain, "1.0");
	bindParameter("va_gain", m_va_gain, "1.0");

	bindParameter("offset_human", m_offset_human, "0.5");
	bindParameter("forget_time", m_forget_time, "1.0");
	// </rtc-template>


	pImages[0] = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	pImages[1] = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	//pImages[1] = new cv::Mat(WIDTH, HEIGHT, CV_8UC3);


	return RTC::RTC_OK;
}


RTC::ReturnCode_t HumanFollower::onFinalize()
{
	endFlag = true;
	return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t HumanFollower::onStartup(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HumanFollower::onShutdown(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

static int counter;

const int MAX_OBJECTS = 12;
CvScalar colors[MAX_OBJECTS] = {
	CV_RGB(255, 0, 0),
	CV_RGB(0, 255, 0),
	CV_RGB(0, 0, 255),

	CV_RGB(255, 255, 0),
	CV_RGB(0, 255, 255),
	CV_RGB(255, 0, 255),

	CV_RGB(255, 125, 0),
	CV_RGB(0, 255, 125),
	CV_RGB(125, 0, 255),

	CV_RGB(255, 125, 125),
	CV_RGB(125, 255, 125),
	CV_RGB(125, 125, 255),
};

const double meter_by_pixel = 0.008;

int to_pixel(int max, const double x) {
	return max / 2 - x / meter_by_pixel;
}

CvPoint toCvPoint(const Point& p) {
	return cvPoint(to_pixel(WIDTH, p.y), to_pixel(HEIGHT, p.x));
}


inline Point range_to_point(const double range, const double angle) {
	return Point(range * cos(angle), range * sin(angle));
}



static bool _not_found = false;
static coil::TimeValue _not_found_start_time;
static double _stop_not_found_time = 1.0;


static int history_size = 20;

static bool initialized = false;
static Human trackingHuman(Object(Point(-10000, -10000)));

#define CV_SHOW


RTC::ReturnCode_t HumanFollower::resetFollowing(void)
{
	m_trackingHumanHistory.clear();
	initialized = false;
	trackingHuman = Human(Object(Point(-10000, -10000)));
	return RTC::RTC_OK;
}


RTC::ReturnCode_t HumanFollower::onActivated(RTC::UniqueId ec_id)
{
#ifdef WIN32
	cv::namedWindow("HumanFollower", CV_WINDOW_AUTOSIZE);
#else 
#ifdef Linux
	cv::namedWindow("HumanFollower", CV_WINDOW_AUTOSIZE);
#endif

#endif
	return resetFollowing();
}


RTC::ReturnCode_t HumanFollower::onDeactivated(RTC::UniqueId ec_id)
{
#ifdef WIN32
	cv::destroyWindow("HumanFollower");
#else
#ifdef Linux
	cv::destroyWindow("HumanFollower");
#endif
#endif

	m_velocity.data.vx = m_velocity.data.vy = m_velocity.data.va = 0;
	setTimestamp(m_velocity);
	m_velocityOut.write();
	coil::usleep(1 * 1000 * 1000);
	setTimestamp(m_velocity);
	m_velocityOut.write();

	return resetFollowing();
}


void HumanFollower::detectObjectsFromRange(void) {

#ifdef CV_SHOW
	uint32_t bufferImageCount = imageCount == 1 ? 0 : 1;
	IplImage* image = pImages[bufferImageCount];
#endif

	/// Clear detected object memory.
	for (int i = 0; i < m_range.ranges.length(); i += m_laser_detect_step) { // for each laser (step can be configured)
		double angle = m_range.config.minAngle + i * m_range.config.angularRes;

		/// If laser detected landmark is out of range....
		if (angle < m_min_detect_angle
			|| angle > m_max_detect_angle
			|| m_range.ranges[i] < m_min_detect_range
			|| m_range.ranges[i] > m_max_detect_range) {

#ifdef CV_SHOW
			cvLine(image, cvPoint(WIDTH / 2, HEIGHT / 2), toCvPoint(p), CV_RGB(70, 70, 70), 1, 8, 0);
#endif
			continue; // Do nothing
		}


		Point p = range_to_point(m_range.ranges[i], angle);

		/// Object Divider
		/// If the neiboring laser distance is similar to the target laser, target and neibor are detecting the same object.
		bool newObject = true;
		for (int j = 1; j <= m_neibor_laser_index_range; j++) {
			if (i - j > 0) {
				if (fabs(m_range.ranges[i - j] - m_range.ranges[i]) < m_neibor_laser_distance_range*j) {
					newObject = false;
				}
			}
		}


		if (newObject || m_detectedObjects.size() == 0) {
			m_detectedObjects.push_back(Object(p));
		}
		else {
			m_detectedObjects[m_detectedObjects.size() - 1].points.push_back(p);
		}
#ifdef CV_SHOW
		cvLine(image, cvPoint(WIDTH / 2, HEIGHT / 2), toCvPoint(p), colors[m_detectedObjects.size() % MAX_OBJECTS], 1, 8, 0);
#endif		
	}


	/// 物体の発見数
	std::cout << "Objects = " << m_detectedObjects.size() << std::endl;
}

void HumanFollower::detectLegsFromObjects(void) {
	for (auto o : m_detectedObjects) {
		if (m_min_leg_width < width(o) && width(o) < m_max_leg_width) {
			m_detectedLegs.push_back(o);
		}
	}
}

void HumanFollower::detectHumansFromObjects(void) {
	for (auto o : m_detectedObjects) {
		if (m_min_body_width < width(o) && width(o) < m_max_body_width) {
			auto h = Human(center(o), width(o) / 2);
			m_detectedHumans.push_back(h);
		}
	}
}

void HumanFollower::detectHumansFromLegs(void)
{

	if (m_detectedLegs.size() == 1) {
		m_detectedHumans.push_back(Human(m_detectedLegs[0]));
		return;
	}

	///double max_distance_legs = 0.5;
	for (int i = 0; i < m_detectedLegs.size(); i++) {
		bool foundPair = false;
		for (int j = 0; j < m_detectedLegs.size(); j++) {
			if (i == j) continue;
			
			if (distance(center(m_detectedLegs[i]), center(m_detectedLegs[j])) < m_max_distance_between_legs) {
				auto h = Human(m_detectedLegs[i], m_detectedLegs[j]);
				m_detectedHumans.push_back(h);
				foundPair = true;
			}
			
		}

		if (!foundPair) {
			m_detectedHumans.push_back(Human(m_detectedLegs[i]));
		}
	}

	/// ここで初めて人間の場所が特定されたのでセレクトをする
	//std::cout << "Humans = " << humans.size() << std::endl;


}

bool HumanFollower::findTrackingHuman() {
	if (m_detectedHumans.size() > 0 && m_trackingHumanHistory.size() == 0) {
		return findFirstTrackingHuman();
	}
	else if (m_detectedHumans.size() > 0) {
		return findNextTrackingHuman();
	}
}

bool HumanFollower::findFirstTrackingHuman()
{
	Human *pTrackingHuman = NULL;
	double maxY = 10000;
	for (auto h : m_detectedHumans) {
		std::cout << "H:" << h.point.x << "," << h.point.y << std::endl;
		if (fabs(h.point.y) < maxY) {
			maxY = fabs(h.point.y);
			pTrackingHuman = &h;
		}
	}

	if (pTrackingHuman) {
		m_trackingHumanHistory.push_back(*pTrackingHuman);
		return true;
	}
	return false;
}

bool HumanFollower::findNextTrackingHuman() {
	double max_distance = m_maxTravelHuman;
	Human* pTrackingHuman = NULL;
	for (auto h : m_detectedHumans) {
		double d = distance(h.point, (*(m_trackingHumanHistory.rbegin())).point);
		std::cout << "(" << d << "),";
		if (d < max_distance) {
			max_distance = d;
			pTrackingHuman = &h;
		}
	}
	if (pTrackingHuman) {
		m_trackingHumanHistory.push_back(*pTrackingHuman);
		return true;
	}
	return false;
}

RTC::ReturnCode_t HumanFollower::onExecute(RTC::UniqueId ec_id)
{
	/// When Range Data is reached,
	if (m_rangeIn.isNew()) {
		m_rangeIn.read(); // Read Data to buffer.

#ifdef CV_SHOW
		uint32_t bufferImageCount = imageCount == 1 ? 0 : 1;
		IplImage* image = pImages[bufferImageCount];
		cvZero(image);
#endif
		m_detectedObjects.clear();
		detectObjectsFromRange();

		m_detectedLegs.clear();
		detectLegsFromObjects();

		m_detectedHumans.clear();
		detectHumansFromObjects();

		detectHumansFromLegs();

		if (!findTrackingHuman()) {

		}
		


		if (!updated) {// トラッキング先の人間が見つからない
			std::cout << "Not Found" << std::endl;
			if (!_not_found) {
				_not_found = true;
				_not_found_start_time = coil::gettimeofday();
			}

			auto current = coil::gettimeofday();
			double interval = current - _not_found_start_time;
			if (interval >= m_forget_time) {

				trackingHumanHistory.clear();
				initialized = false;
				trackingHuman = Human(Object(Point(-10000, -10000)));
			}
		}

		double maxDistanceHumanWalk = 0.5;
		//    if (distance(trackingHuman.point, nextTrackingHuman.point) < maxDistanceHumanWalk) {
		trackingHuman = nextTrackingHuman;
		//    }

		trackingHumanHistory.push_back(trackingHuman);
		if (trackingHumanHistory.size() > history_size) {
			trackingHumanHistory.erase(trackingHumanHistory.begin());
		}

#ifdef CV_SHOW
		for (auto leg : legs) {
			// cvCircle(image, toCvPoint(center(leg)), width(leg) /2 / meter_by_pixel, colors[legs.size() % MAX_OBJECTS], -1);
		}
		for (auto h : humans) {
			cvCircle(image, toCvPoint(h.point), h.radius / meter_by_pixel, colors[humans.size() % MAX_OBJECTS]);
		}


		if (initialized && objects.size() > 0) {
			const int L = 50;
			CvPoint hp = toCvPoint(trackingHuman.point);
			cvLine(image, cvPoint(hp.x, hp.y + L), cvPoint(hp.x, hp.y - L), CV_RGB(255, 255, 255), 1, 4, 0);
			cvLine(image, cvPoint(hp.x + L, hp.y), cvPoint(hp.x - L, hp.y), CV_RGB(255, 255, 255), 1, 4, 0);
			cvLine(image, cvPoint(hp.x, hp.y), cvPoint(WIDTH / 2, HEIGHT / 2), CV_RGB(255, 255, 255), 1, 4, 0);

			Human buf = trackingHumanHistory[0];
			for (int i = 1; i < trackingHumanHistory.size(); i++) {
				CvPoint p1 = toCvPoint(buf.point);
				CvPoint p2 = toCvPoint(trackingHumanHistory[i].point);
				cvLine(image, cvPoint(p1.x, p1.y), cvPoint(p2.x, p2.y), CV_RGB(255, 255, 255), 1, 4, 0);
				buf = trackingHumanHistory[i];
			}
		}
		// 軸を書く
		cvLine(image, cvPoint(WIDTH / 2, 0), cvPoint(WIDTH / 2, HEIGHT), CV_RGB(255, 255, 255), 1, 4, 0);
		cvLine(image, cvPoint(0, HEIGHT / 2), cvPoint(WIDTH, HEIGHT / 2), CV_RGB(255, 255, 255), 1, 4, 0);
		// レーダーサークルを書く
		cvCircle(image, cvPoint(WIDTH / 2, HEIGHT / 2), 1.0 / meter_by_pixel, CV_RGB(255, 255, 255));
		cvCircle(image, cvPoint(WIDTH / 2, HEIGHT / 2), 2.0 / meter_by_pixel, CV_RGB(255, 255, 255));
		cvCircle(image, cvPoint(WIDTH / 2, HEIGHT / 2), 3.0 / meter_by_pixel, CV_RGB(255, 255, 255));
		imageCount = bufferImageCount;


#ifdef WIN32

		cvShowImage("HumanFollower", pImages[imageCount]);
		//coil::usleep(100*1000);
		///std::cout << "update[" << imageCount << "]" << std::endl;
		cv::waitKey(10);
#else
#ifdef Linux

		cvShowImage("HumanFollower", pImages[imageCount]);
		//coil::usleep(100*1000);
		///std::cout << "update[" << imageCount << "]" << std::endl;
		cv::waitKey(10);
#endif
#endif
#endif
		// human Tracked.
		if (humans.size() > 0) {
			std::cout << "[TrackingHuman] " << trackingHuman.point.x << ", " << trackingHuman.point.y << std::endl;
			double dx = trackingHuman.point.x;
			double dy = trackingHuman.point.y;

			double offset = m_offset_human;
			double dx_gain = 0.5;
			double da_gain = 0.8;

			double distance = sqrt(dx * dx + dy * dy) - offset;
			double angle = atan2(dy, dx);
			// std::cout << "dth = " << angle << std::endl;
			double vx = distance > 0 ? distance * distance * dx_gain : 0;
			double vy = 0;
			double va = angle * da_gain;

			m_velocity.data.vx = vx;
			m_velocity.data.vy = vy;
			m_velocity.data.va = va;
			setTimestamp(m_velocity);
			m_velocityOut.write();
		}

	}

	return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t HumanFollower::onAborting(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HumanFollower::onError(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HumanFollower::onReset(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HumanFollower::onStateUpdate(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HumanFollower::onRateChanged(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/



extern "C"
{

	void HumanFollowerInit(RTC::Manager* manager)
	{
		coil::Properties profile(humanfollower_spec);
		manager->registerFactory(profile,
			RTC::Create<HumanFollower>,
			RTC::Delete<HumanFollower>);
	}

};


