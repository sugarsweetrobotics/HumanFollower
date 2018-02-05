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
    "type_name",         "HumanFollower",
    "description",       "Human Follow Commander for Mobile Robots",
    "version",           "1.0.0",
    "vendor",            "SUGAR SWEET ROBOTICS",
    "category",          "MobileRobot",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.vx_gain", "1.0",
    "conf.default.va_gain", "1.0",
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
  CV_RGB(255,   0,   0),
  CV_RGB(  0, 255,   0),
  CV_RGB(  0,   0, 255),

  CV_RGB(255, 255,   0),
  CV_RGB(  0, 255, 255),
  CV_RGB(255,   0, 255),

  CV_RGB(255, 125,   0),
  CV_RGB(  0, 255, 125),
  CV_RGB(125,   0, 255),

  CV_RGB(255, 125, 125),
  CV_RGB(125, 255, 125),
  CV_RGB(125, 125, 255),
};

RTC::ReturnCode_t HumanFollower::onActivated(RTC::UniqueId ec_id)
{
#ifdef WIN32
	cv::namedWindow("HumanFollower", CV_WINDOW_AUTOSIZE);
#else 
#ifdef Linux
	cv::namedWindow("HumanFollower", CV_WINDOW_AUTOSIZE);
#endif

#endif
  return RTC::RTC_OK;
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
	return RTC::RTC_OK;
}


struct Point {
  double x;
  double y;

  Point(const double x, const double y): x(x), y(y) {}
  Point(const Point& p): x(p.x), y(p.y) {}

};

bool operator==(const Point& p1, const Point& p2) {
  return p1.x == p2.x && p1.y == p2.y;
}

struct Object {
  std::vector<Point> points;

  Object() {}

  Object(const Point& p) {
    points.push_back(p);
  }

  Object(const Object& o) {
    points = o.points;
  }
};

bool operator==(const Object& o1, const Object& o2) {
  return o1.points == o2.points;
}

Point average(const Point& p0, const Point& p1) {
  return Point((p0.x + p1.x)/2, (p0.y + p1.y)/2);
}

Point center(const Object& o) {
  if (o.points.size() < 2) return Point(0,0);
  return average(o.points[0], o.points[o.points.size()-1]);
}

double distance(const Point& p0, const Point& p1) {
  double dx = p0.x - p1.x;
  double dy = p0.y - p1.y;
  return sqrt(dx*dx + dy*dy);
}

double width(const Object& o) {
  if (o.points.size() == 0) return 0;
  double dx = (o.points[0].x - o.points[o.points.size()-1].x);
  double dy = (o.points[0].y - o.points[o.points.size()-1].y);
  return sqrt(dx*dx + dy*dy);
}

Point range_to_point(const double range, const double angle) {
  return Point(range * cos(angle), range * sin(angle));
}

const double meter_by_pixel = 0.008;
int to_pixel(int max, const double x) { 
  return max / 2 - x / meter_by_pixel;
}

CvPoint toCvPoint(const Point& p) {
  return cvPoint(to_pixel(WIDTH, p.y), to_pixel(HEIGHT, p.x));
}



struct Human {
  Point point;
  double radius;
  Object leg1;
  Object leg2;
  Human() : point(0,0), radius(0) {};
  Human(const Object&leg1, const Object& leg2): leg1(leg1), leg2(leg2),
						point(average(center(leg1), center(leg2))),
						radius(distance(center(leg1), center(leg2))/2) {
  }

  Human(const Object& leg) : leg1(leg), point(center(leg)), radius(0.30) {
  }

  Human(const Point& point, const double radius): leg1(Object(point)), point(point), radius(radius) {}

  Human(const Human& human): leg1(human.leg1), leg2(human.leg2), point(human.point), radius(human.radius) {} 

  void operator=(const Human& human) {
    leg1 = (human.leg1), leg2 = (human.leg2), point = (human.point), radius = (human.radius); }
};

static std::vector<Object> objects;
static std::vector<Object> legs;
static std::vector<Human> humans;

static bool initialized = false;
static Human trackingHuman(Object(Point(-10000, -10000)));

#define CV_SHOW

RTC::ReturnCode_t HumanFollower::onExecute(RTC::UniqueId ec_id)
{
  if (m_rangeIn.isNew()) {
    m_rangeIn.read();
    //std::cout << "Range Data Received. (length=" << m_range.ranges.length() << ")" << std::endl;    
    double maxDetection = 3.5;

#ifdef CV_SHOW
    uint32_t bufferImageCount = imageCount == 1 ? 0 : 1;
    IplImage* image = pImages[bufferImageCount];
    cvZero(image);
#endif

    objects.clear();
    for (int i = 0; i < m_range.ranges.length(); i+=1) {
      Point p = range_to_point(m_range.ranges[i], m_range.config.minAngle + i * m_range.config.angularRes);
      
      bool findObject = m_range.ranges[i] < maxDetection;
      bool newObject = true;
      if (i != 0) {
	for(int j = 1;j <= 1;j++) {
	  if (i - j > 0) {
	    if (fabs(m_range.ranges[i-j] - m_range.ranges[i]) < 0.05*j) {
	      newObject = false;
	    }
	  }
	}
      }

      
      if (findObject) {

	if (newObject || objects.size() == 0) {
	  objects.push_back(Object(p));
	} else {
	  objects[objects.size()-1].points.push_back(p);
	}
      }
      
      //auto color = m_range.ranges[i] > maxDetection ? CV_RGB(70, 70, 70) : colors[objects.size() % MAX_OBJECTS];
#ifdef CV_SHOW
      if (findObject) {
	cvLine(image, cvPoint(WIDTH/2, HEIGHT / 2), toCvPoint(p), colors[objects.size() % MAX_OBJECTS], 1, 8, 0);
      } else {
	cvLine(image, cvPoint(WIDTH / 2, HEIGHT / 2), toCvPoint(p), CV_RGB(70, 70, 70), 1, 8, 0);
      }
#endif
    }
  
    /// 物体の発見数
    // std::cout << "Objects = " << objects.size() << std::endl;
    legs.clear();
    humans.clear();
    for (auto o: objects) {
      double min_width_leg = 0.05;
      double max_width_leg = 0.20;
      double min_width_body = max_width_leg;
      double max_width_body = 0.5;
      
      if (min_width_leg < width(o) && width(o) < max_width_leg) {
	legs.push_back(o);
      }

      if (min_width_body < width(o) && width(o) < max_width_body) {
	auto h = Human(center(o), width(o)/2);
	humans.push_back(h);
      }	
    }

    /// ここで足を検出
    //std::cout << "Legs = " << legs.size() << std::endl;
    //for (auto leg : legs) {
    //  auto c = center(leg);
    //}

    if (legs.size() == 1) {
      humans.push_back(Human(legs[0]));
    }

    for (int i = 1;i < legs.size();i++) {
      double max_distance_legs = 0.5;
      if (distance(center(legs[i-1]), center(legs[i])) < max_distance_legs) {
	auto h = Human(legs[i-1], legs[i]);

	humans.push_back(h);
      } else {
	///humans.push_back(Human(legs[i-1]));
      }
    }

    /// ここで初めて人間の場所が特定されたのでセレクトをする
    std::cout << "Humans = " << humans.size() << std::endl;
    double maximumHumanTravel = 0.5;
    int updated = 0;
    Human nextTrackingHuman = trackingHuman;
    if (humans.size() > 0 && !initialized) { /// 最初のトラッキングの場合
      double maxY = 10000;
      for(auto h : humans) {
	std::cout << "H:" << h.point.x << "," << h.point.y << std::endl;
	if (fabs(h.point.y) < maxY) {
	  maxY = fabs(h.point.y);
	  trackingHuman = h;
	  nextTrackingHuman = h;
	  updated ++;
	}
      }
      initialized = true;
    } else if (humans.size() > 0) {
      double max_distance = maximumHumanTravel;
      for(auto h : humans) {
	double d = distance(h.point, trackingHuman.point);
	std::cout << "(" << d << "),";
	if (d < max_distance) {
	  max_distance = d;
	  nextTrackingHuman = h;
	  updated++;
	}
      }
      std::cout << std::endl;
    }


    if (updated == 0) {// トラッキング先の人間が見つからない
      std::cout << "Not Found" << std::endl;
    }
    
    double maxDistanceHumanWalk = 0.5;
    //    if (distance(trackingHuman.point, nextTrackingHuman.point) < maxDistanceHumanWalk) {
      trackingHuman = nextTrackingHuman;
      //    }


    
#ifdef CV_SHOW
    for(auto leg: legs) {
      // cvCircle(image, toCvPoint(center(leg)), width(leg) /2 / meter_by_pixel, colors[legs.size() % MAX_OBJECTS], -1);
    }
    for(auto h : humans) {
      cvCircle(image, toCvPoint(h.point), h.radius / meter_by_pixel, colors[humans.size() % MAX_OBJECTS]);
    }
    

    if (initialized && objects.size() > 0) {
      const int L = 50;
      CvPoint hp = toCvPoint(trackingHuman.point);
      cvLine(image, cvPoint(hp.x, hp.y+L), cvPoint(hp.x, hp.y-L), CV_RGB(255, 255, 255), 1, 4, 0);
      cvLine(image, cvPoint(hp.x+L, hp.y), cvPoint(hp.x-L, hp.y), CV_RGB(255, 255, 255), 1, 4, 0);
      cvLine(image, cvPoint(hp.x, hp.y), cvPoint(WIDTH/2, HEIGHT/2), CV_RGB(255, 255, 255), 1, 4, 0);
    }
    // 軸を書く
    cvLine(image, cvPoint(WIDTH / 2, 0), cvPoint(WIDTH / 2, HEIGHT), CV_RGB(255, 255, 255), 1, 4, 0);
    cvLine(image, cvPoint(0, HEIGHT / 2), cvPoint(WIDTH, HEIGHT / 2), CV_RGB(255, 255, 255), 1, 4, 0);
    // レーダーサークルを書く
    cvCircle(image, cvPoint(WIDTH/2, HEIGHT/2), 1.0 / meter_by_pixel, CV_RGB(255, 255, 255)); 
    cvCircle(image, cvPoint(WIDTH/2, HEIGHT/2), 2.0 / meter_by_pixel, CV_RGB(255, 255, 255)); 
    cvCircle(image, cvPoint(WIDTH/2, HEIGHT/2), 3.0 / meter_by_pixel, CV_RGB(255, 255, 255)); 
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

      double offset = 1.0;
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


