/*!
 * \file
 * \brief
 * \author Patryk Wasowski
 */

#include <memory>
#include <string>

#include "FloatArrayProxy.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


namespace Processors {
namespace FloatArrayProxy {

FloatArrayProxy::FloatArrayProxy(const std::string & name) :
		Base::Component(name) , 
		ros_topic_name("ros_topic_name", std::string("float_array")), 
		ros_namespace("ros_namespace", std::string("discode")) {
	registerProperty(ros_topic_name);
	registerProperty(ros_namespace);

}

FloatArrayProxy::~FloatArrayProxy() {
}

void FloatArrayProxy::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_data", &in_data);
	// Register handlers
	registerHandler("onNewData", boost::bind(&FloatArrayProxy::onNewData, this));
	addDependency("onNewData", &in_data);

}

bool FloatArrayProxy::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	pub = nh->advertise<std_msgs::Float32>(ros_topic_name, 1000);
	
	return true;
}

bool FloatArrayProxy::onFinish() {
	return true;
}

bool FloatArrayProxy::onStop() {
	return true;
}

bool FloatArrayProxy::onStart() {
	return true;
}

void FloatArrayProxy::onNewData() {
	std_msgs::Float32 msg;
//	msg.data = in_data.read();
//	std::cout << in_data.read();
    msg.data = 5.0;
	pub.publish(msg);
	ros::spinOnce();
}



} //: namespace FloatArrayProxy
} //: namespace Processors
