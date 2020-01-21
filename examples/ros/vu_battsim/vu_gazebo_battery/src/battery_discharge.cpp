#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"
#include "battery_discharge.hh"
//#include <kobuki_msgs/MotorPower.h>		DISABLED SINCE VU ONLY NEEDS BASIC BATT SIMULATION
#include "std_msgs/Float64.h"
#include "ROS_debugging.h"

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


enum power{
    OFF = 0,
    ON = 1
};

template <typename T>
T max(T x, T y)
{
    return x < y ? y : x;
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin);

BatteryPlugin::BatteryPlugin()
{
    this->c = 0.0;
    this->r = 0.0;
    this->tau = 0.0;

    this->e0 = 0.0;
    this->e1 = 0.0;

    this->q0 = 0.0;
    this->q = 0.0;
    this->qt = 0.0;

    this->iraw = 0.0;
    this->ismooth = 0.0;

    #ifdef BATTERY_DEBUG
        gzdbg << VU "Constructed BatteryPlugin and initialized parameters. \n";
    #endif

    ROS_GREEN_STREAM(VU "Battery is constructed.");
}

BatteryPlugin::~BatteryPlugin()
{
    #ifdef BATTERY_DEBUG
        gzdbg << VU "Destructing BatteryPlugin and removing ROS node. \n";
    #endif
    this->rosNode->shutdown();
}

void BatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    #ifdef BATTERY_DEBUG
        gzdbg << "Loading the BatteryPlugin \n";
    #endif

    // check if the ros is up!
    if (!ros::isInitialized()){
        ROS_GREEN_STREAM(VU "Initializing ROS...");
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, _sdf->Get<std::string>("ros_node"), ros::init_options::NoSigintHandler);
    }

    this->model = _model;
    this->world = _model->GetWorld();

    this->sim_time_now = this->world->GetSimTime().Double();

    // Create ros node and publish stuff there!
    this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("ros_node")));
    if (this->rosNode->ok())
    {
        ROS_GREEN_STREAM(VU "ROS node is up");
    }

    // Publish a topic for motor power and charge level
    //this->motor_power = this->rosNode->advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
    this->charge_state = this->rosNode->advertise<std_msgs::Float64>("/mobile_base/commands/charge_level", 1);
    this->charge_state_mwh = this->rosNode->advertise<std_msgs::Float64>("/mobile_base/commands/charge_level_mwh", 1);

    this->set_charging = this->rosNode->advertiseService(this->model->GetName() + "/set_charging", &BatteryPlugin::SetCharging, this);
    this->set_charging_rate = this->rosNode->advertiseService(this->model->GetName() + "/set_charge_rate", &BatteryPlugin::SetChargingRate, this);
    this->set_charge = this->rosNode->advertiseService(this->model->GetName() + "/set_charge", &BatteryPlugin::SetCharge, this);
    this->set_coefficients = this->rosNode->advertiseService(this->model->GetName() + "/set_model_coefficients", &BatteryPlugin::SetModelCoefficients, this);

    std::string linkName = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(linkName);

    this->e0 = _sdf->Get<double>("constant_coef");
    this->e1 = _sdf->Get<double>("linear_coef");
    this->q0 = _sdf->Get<double>("initial_charge");
    this->qt = _sdf->Get<double>("charge_rate");
    this->c = _sdf->Get<double>("capacity");
    this->r = _sdf->Get<double>("resistance");
    this->tau = _sdf->Get<double>("smooth_current_tau");

    std::string batteryName = _sdf->Get<std::string>("battery_name");

    if (this->link->BatteryCount() > 0) {
        // Creates the battery
        this->battery = this->link->Battery(batteryName);
        ROS_GREEN_STREAM(VU "Created a battery");
    }
    else
    {
        ROS_RED_STREAM(VU "There is no battery specification in the link: check your .world file");
    };


    // Specifying a custom update function
    this->battery->SetUpdateFunc(std::bind(&BatteryPlugin::OnUpdateVoltage, this, std::placeholders::_1));

    this->sim_time_now = this->world->GetSimTime().Double();

    #ifdef BATTERY_DEBUG
        gzdbg << VU "Loaded the BatteryPlugin at time:" << this->sim_time_now << "\n";
    #endif

    ROS_GREEN_STREAM(VU "Plugin is fully loaded.");
}

// This is for in initialization purposes and is called once after Load
// So explanation about Gazebo Init are here: http://playerstage.sourceforge.net/doc/Gazebo-manual-0.5-html/plugin_models.html
void BatteryPlugin::Init()
{
    ROS_GREEN_STREAM(VU "Init Battery");
    this->q = this->q0;
    this->charging = false;
}

void BatteryPlugin::Reset()
{
    this->iraw = 0.0;
    this->ismooth = 0.0;
    this->Init();
}

double BatteryPlugin::OnUpdateVoltage(const common::BatteryPtr &_battery)
{
    double dt = this->world->GetPhysicsEngine()->GetMaxStepSize();
    double totalpower = 0.0;
    double k = dt / this->tau;

    if (fabs(_battery->Voltage())<1e-3)
        return 0.0;

    for (auto powerLoad : _battery->PowerLoads())
        totalpower += powerLoad.second;

    // current = power(Watts)/Voltage
    this->iraw = totalpower / _battery->Voltage();

    this->ismooth = this->ismooth + k * (this->iraw - this->ismooth);

    if (!this->charging)
    {
        this->q = this->q - GZ_SEC_TO_HOUR(dt * this->ismooth);
    }
    else
    {
        this->q = this->q + GZ_SEC_TO_HOUR(dt * this->qt);
    }

    this->sim_time_now = this->world->GetSimTime().Double();

    #ifdef BATTERY_DEBUG
        gzdbg << VU "Current charge:" << this->q << ", at:" << this->sim_time_now << "\n";
    #endif
    //    ROS_INFO_STREAM(this->q);

    this->et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

    #ifdef BATTERY_DEBUG
        gzdbg << VU "Current voltage:" << this->et << ", at:" << this->sim_time_now << "\n";
    #endif

    //Turn off the motor
    if (this->q <= 0)
    {
        this->sim_time_now = this->world->GetSimTime().Double();

        #ifdef BATTERY_DEBUG
            gzdbg << VU "Out of juice at:" << this->sim_time_now << "\n";
        #endif

        this->q = 0;
	ROS_RED_STREAM(VU "Battery ran out of charge");
    //====DISABLED SINCE VU ONLY NEEDS BASIC BATT SIMULATION====
    //    kobuki_msgs::MotorPower power_msg;		Not needed
    //    power_msg.state = power::OFF;
    //    lock.lock();
    //    this->motor_power.publish(power_msg);		Not available anymore
    //    lock.unlock();
    }
    else if (this->q >= this->c)
    {
        this->q = this->c;
    }

    std_msgs::Float64 charge_msg, charge_msg_mwh;
    charge_msg.data = this->q;
    charge_msg_mwh.data = this->q * 1000 * this-> et;

    lock.lock();
    this->charge_state.publish(charge_msg);
    this->charge_state_mwh.publish(charge_msg_mwh);
    lock.unlock();

    return et;
}

bool BatteryPlugin::SetCharging(brass_gazebo_battery::SetCharging::Request& req,
                                brass_gazebo_battery::SetCharging::Response& res)
{
    lock.lock();
    this->charging = req.charging;
    if (this->charging) {
        #ifdef BATTERY_DEBUG
                gzdbg << VU "Bot is charging" << "\n";
        #endif
        ROS_GREEN_STREAM(VU "Bot is charging");
    }
    else{
        #ifdef BATTERY_DEBUG
                gzdbg << VU "Bot disconnected from the charging station" << "\n";
        #endif
        ROS_GREEN_STREAM(VU "Bot disconnected from the charging station");
    }
    lock.unlock();
    res.result = true;
    return true;
}

bool BatteryPlugin::SetChargingRate(brass_gazebo_battery::SetChargingRate::Request& req,
                                brass_gazebo_battery::SetChargingRate::Response& res)
{
    lock.lock();
    this->qt = req.charge_rate;
    #ifdef BATTERY_DEBUG
            gzdbg << VU "Charging rate has been changed to: " << this->qt << "\n";
    #endif
    ROS_GREEN_STREAM(VU "Charging rate has been changed to: " << this->qt);
    lock.unlock();
    res.result = true;
    return true;
}


bool BatteryPlugin::SetCharge(brass_gazebo_battery::SetCharge::Request &req,
                              brass_gazebo_battery::SetCharge::Response &res)
{
    lock.lock();
    if (req.charge <= this->c){
        this->q = req.charge;
        #ifdef BATTERY_DEBUG
            gzdbg << VU "Received charge:" << this->q << "\n";
        #endif
        ROS_GREEN_STREAM(VU "A new charge is set: " << this->q);
    }
    else
    {
        this->q = this->c;
        ROS_RED_STREAM(VU "The charge cannot be higher than the capacity of the battery");
    }
    lock.unlock();
    res.result = true;
    return true;
}

bool BatteryPlugin::SetModelCoefficients(brass_gazebo_battery::SetCoef::Request &req,
                                         brass_gazebo_battery::SetCoef::Response &res)
{
    lock.lock();
    this->e0 = req.constant_coef;
    this->e1 = req.linear_coef;
    #ifdef BATTERY_DEBUG
        gzdbg << VU "Power model is changed, new coefficients (constant, linear):" << this->e0 << this->e1 << "\n";
    #endif
    ROS_GREEN_STREAM(VU "Power model is changed, new coefficients (constant, linear):" << this->e0 << this->e1);
    lock.unlock();
    res.result = true;
    return true;
}
