/*!
 * \file
 * \brief 
 * \author Patryk Wasowski
 */

#ifndef FLOATARRAYPROXY_HPP_
#define FLOATARRAYPROXY_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>
#include "ros/ros.h"

#include "std_msgs/Float32.h"


namespace Processors {
namespace FloatArrayProxy {

/*!
 * \class FloatArrayProxy
 * \brief FloatArrayProxy processor class.
 *
 * 
 */
class FloatArrayProxy: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	FloatArrayProxy(const std::string & name = "FloatArrayProxy");

	/*!
	 * Destructor
	 */
	virtual ~FloatArrayProxy();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams
	Base::DataStreamIn< std::vector<float> > in_data;

	// Output data streams

	// Handlers

	// Properties
	Base::Property<std::string> ros_topic_name;
	Base::Property<std::string> ros_namespace;
	
	ros::Publisher pub;
	ros::NodeHandle * nh;

	
	// Handlers
	void onNewData();

};

} //: namespace FloatArrayProxy
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("FloatArrayProxy", Processors::FloatArrayProxy::FloatArrayProxy)

#endif /* FLOATARRAYPROXY_HPP_ */
