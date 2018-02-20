// -*- C++ -*-
/*!
 * @file  HumanFollower.h
 * @brief Human Follow Commander for Mobile Robots
 * @date  $Date$
 *
 * @author Yuki Suga, Sugar Sweet Robotics
 * @ysuga
 *
 * GPLv3
 *
 * $Id$
 */

#ifndef HUMANFOLLOWER_H
#define HUMANFOLLOWER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
// </rtc-template>

using namespace RTC;

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#ifndef NO_OPENCV
#include <opencv2/opencv.hpp>
#endif


/**
 *
 */
struct Point {
public:
	double x;
	double y;
public:
	Point(const double x, const double y) : x(x), y(y) {}
	Point(const Point& p) : x(p.x), y(p.y) {}
};

/**
 *
 */
inline bool operator==(const Point& p1, const Point& p2) {
	return p1.x == p2.x && p1.y == p2.y;
}

/**
 *
 */
struct Object {
public:
	std::vector<Point> points;

public:
	Object() {}

	Object(const Point& p) {
		points.push_back(p);
	}

	Object(const Object& o) {
		points = o.points;
	}
};

inline bool operator==(const Object& o1, const Object& o2) {
	return o1.points == o2.points;
}



inline Point average(const Point& p0, const Point& p1) {
	return Point((p0.x + p1.x) / 2, (p0.y + p1.y) / 2);
}

inline Point center(const Object& o) {
	if (o.points.size() < 2) return Point(0, 0);
	return average(o.points[0], o.points[o.points.size() - 1]);
}

inline double distance(const Point& p0, const Point& p1) {
	double dx = p0.x - p1.x;
	double dy = p0.y - p1.y;
	return sqrt(dx*dx + dy*dy);
}

inline double width(const Object& o) {
	if (o.points.size() == 0) return 0;
	double dx = (o.points[0].x - o.points[o.points.size() - 1].x);
	double dy = (o.points[0].y - o.points[o.points.size() - 1].y);
	return sqrt(dx*dx + dy*dy);
}

/**
 *
 */
struct Human {
public:
	Point point;
	double radius;
	Object leg1;
	Object leg2;

public:
	Human() : point(0, 0), radius(0) {};
	Human(const Object&leg1, const Object& leg2) : leg1(leg1), leg2(leg2),
		point(average(center(leg1), center(leg2))),
		radius(distance(center(leg1), center(leg2)) / 2) {
	}

	Human(const Object& leg) : leg1(leg), point(center(leg)), radius(0.30) {
	}

	Human(const Point& point, const double radius) : leg1(Object(point)), point(point), radius(radius) {}

	Human(const Human& human) : leg1(human.leg1), leg2(human.leg2), point(human.point), radius(human.radius) {}

	void operator=(const Human& human) {
		leg1 = (human.leg1), leg2 = (human.leg2), point = (human.point), radius = (human.radius);
	}
};

/*!
 * @class HumanFollower
 * @brief Human Follow Commander for Mobile Robots
 *
 * Human Following Behavior Generator for Mobile Robot using
 * Range Data Sensor (like Hokuyo URG).
 * This depends on OpenCV2 for visualization. If you do not need
 * visualization, edit CMakeLists.txt in Root Directory of this
 * component to turn OFF the option USE_CV_VISUALIZATION.
 *
 * Input: RangeData (LiDAR)
 * Output: TimedVelocity2D (for Mobile Robots)
 *
 */
class HumanFollower
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  HumanFollower(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~HumanFollower();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  debug
   * - DefaultValue: 0
   */
  int m_debug;
  /*!
   * 
   * - Name:  vx_gain
   * - DefaultValue: 1.0
   */
  float m_vx_gain;
  /*!
   * 
   * - Name:  va_gain
   * - DefaultValue: 1.0
   */
  float m_va_gain;

  float m_offset_human;

  float m_forget_time;

  int m_laser_detect_step;

  float m_max_detect_range;

  float m_min_detect_range;

  float m_max_detect_angle;

  float m_min_detect_angle;

  int m_neibor_laser_index_range;

  int m_neibor_laser_distance_range;

  float m_min_leg_width;
  float m_max_leg_width;

  float m_min_body_width;
  float m_max_body_width;

  float m_max_distance_between_legs;

  float m_maxTravelHuman;
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::RangeData m_range;
  /*!
   */
  InPort<RTC::RangeData> m_rangeIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedVelocity2D m_velocity;
  /*!
   */
  OutPort<RTC::TimedVelocity2D> m_velocityOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>


	 RTC::ReturnCode_t resetFollowing(void);
	 void detectObjectsFromRange(void);

	 void detectLegsFromObjects(void);

	 void detectHumansFromObjects(void);
	 void detectHumansFromLegs(void);

	 std::vector<Object> m_detectedObjects;
	 std::vector<Human> m_trackingHumanHistory;


	 std::vector<Object> m_detectedLegs;

	 std::vector<Human> m_detectedHumans;

	 bool findTrackingHuman();
	 bool findFirstTrackingHuman();
	 bool findNextTrackingHuman();

};


extern "C"
{
  DLL_EXPORT void HumanFollowerInit(RTC::Manager* manager);
};

#endif // HUMANFOLLOWER_H
