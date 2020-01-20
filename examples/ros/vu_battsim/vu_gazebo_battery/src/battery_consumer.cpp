#include "battery_consumer.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"
#include "ROS_debugging.h"

#define BATTERY_CONSUMER_DEBUG
#define VU "[VU_BATT] >> "

/*	==== Plugin is tested for ROS Kinetic and Gazebo 7 ====

	ORIGINAL CODE: 		https://github.com/pooyanjamshidi/brass_gazebo_battery
	DEVELOPED BY: 		Pooyan Jamshidi			https://github.com/pooyanjamshidi

	MODIFIED BY: 		Stan Swanborn			https://github.com/StanSwanborn
	SUPERVISED BY:		Ivano Malavolta			https://github.com/iivanoo
	COMMISSIONED BY: 	Vrije Universiteit Amsterdam	Software Engineering and Green IT

	CODE USED IS PROVIDED UNDER THE MIT LICENSE:

	The MIT License (MIT)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryConsumerPlugin);

BatteryConsumerPlugin::BatteryConsumerPlugin() : consumerId(-1)
{
}

BatteryConsumerPlugin::~BatteryConsumerPlugin()
{
    if (this->battery && this->consumerId !=-1)
        this->battery->RemoveConsumer(this->consumerId);
}

void BatteryConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    #ifdef CONSUMER_DEBUG
        gzdbg << VU "started loading consumer \n";
    #endif

    // check if the ros is up!
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, _sdf->Get<std::string>("ros_node"), ros::init_options::NoSigintHandler);
    }

    this->model = _model;
    this->world = _model->GetWorld();

    std::string linkName = _sdf->Get<std::string>("link_name");
    this->link = _model->GetLink(linkName);

    // Create battery
    std::string batteryName = _sdf->Get<std::string>("battery_name");
    this->battery = this->link->Battery(batteryName);

    // Add consumer and sets its power load
    this->powerLoad = _sdf->Get<double>("power_load");
    this->consumerId = this->battery->AddConsumer();
    this->battery->SetPowerLoad(this->consumerId, powerLoad);

    // Create ros node and publish stuff there!
    this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("ros_node")));

    this->set_power_load = this->rosNode->advertiseService(this->model->GetName() + "/set_power_load", &BatteryConsumerPlugin::SetConsumerPowerLoad, this);

    #ifdef CONSUMER_DEBUG
        gzdbg << VU "Battery consumer loaded \n";
    #endif

    ROS_GREEN_STREAM(VU "Battery consumer is loaded");

}

void BatteryConsumerPlugin::Init()
{
#ifdef CONSUMER_DEBUG
    gzdbg << VU "consumer is initialized \n";
#endif
    ROS_GREEN_STREAM(VU "Battery consumer is initialized");
}

void BatteryConsumerPlugin::Reset()
{
#ifdef CONSUMER_DEBUG
    gzdbg << VU "consumer is reset \n";
#endif
    ROS_GREEN_STREAM(VU "Battery consumer is reset");
}

bool BatteryConsumerPlugin::SetConsumerPowerLoad(brass_gazebo_battery::SetLoad::Request &req,
                                                 brass_gazebo_battery::SetLoad::Response &res)
{
    lock.lock();
    double load = this->powerLoad;
    this->powerLoad = req.power_load;
    this->battery->SetPowerLoad(this->consumerId, this->powerLoad);

    #ifdef BATTERY_CONSUMER_DEBUG
        gzdbg << VU "Power load of consumer has changed from:" << load << ", to:" << this->powerLoad << "\n";
    #endif
    ROS_GREEN_STREAM(VU "Power load of consumer has changed to: " << this->powerLoad);

    lock.unlock();
    res.result = true;
    return true;
}
