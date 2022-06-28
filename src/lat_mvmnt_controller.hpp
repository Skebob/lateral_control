#ifndef LAT_MVMNT_CONTROLLER__
#define LAT_MVMNT_CONTROLLER__

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TwistStamped.h>

#include <automotive_platform_msgs/SteerMode.h>
#include <automotive_platform_msgs/SpeedMode.h>
#include <automotive_platform_msgs/GearCommand.h>
#include <automotive_platform_msgs/TurnSignalCommand.h>
#include <automotive_platform_msgs/VelocityAccelCov.h>
#include <automotive_platform_msgs/CurvatureFeedback.h>
#include <automotive_platform_msgs/ThrottleFeedback.h>
#include <automotive_platform_msgs/BrakeFeedback.h>
#include <automotive_platform_msgs/GearFeedback.h>
#include <automotive_navigation_msgs/ModuleState.h>
#include <automotive_platform_msgs/SteeringFeedback.h>

#include "pacmod_msgs/PositionWithSpeed.h"
#include "pacmod_msgs/PacmodCmd.h"

/* we need this from SSC
    steer_mode_pub_ = nh_.advertise<automotive_platform_msgs::SteerMode>("ssc/arbitrated_steering_commands", 10);
    speed_mode_pub_ = nh_.advertise<automotive_platform_msgs::SpeedMode>("ssc/arbitrated_speed_commands", 10);
    turn_signal_pub_ = nh_.advertise<automotive_platform_msgs::TurnSignalCommand>("ssc/turn_signal_command", 10);
    gear_pub_ = nh_.advertise<automotive_platform_msgs::GearCommand>("ssc/gear_select", 1, true);
*/

/* PACmod need this from us 
    [message type] | [topic]
    can_msgs/Frame | can_tx
    pacmod_msgs/PacmodCmd | as_rx/accel_cmd
    pacmod_msgs/PacmodCmd | as_rx/brake_cmd
    pacmod_msgs/PacmodCmd | as_rx/shift_cmd
    pacmod_msgs/PositionWithSpeed | as_rx/steer_cmd
    pacmod_msgs/PacmodCmd | as_rx/turn_cmd
    std_msgs/Bool | as_rx/enable
*/


// class decleration 
class LateralControl{
    public:
        LateralControl(ros::NodeHandle* nh);

        void callback_steering(const automotive_platform_msgs::SteerMode::ConstPtr& msg);

    private:   
        //PRIVATE CONST ATTRIBUTES
        const float wheelbase = 2.565; // wheelbase is 256.6cm (2.565m) source: https://gem.polaris.com/en-us/e4/specs/
        //PRIVATE VARIABLES
        float curvature_; // 1/m
        float curvature_limit_; // 1/m/s
        float wheelAngle_;
        float steeringAngle_;

        // subscribers to SSC interface
        ros::Subscriber steer_mode_sub_;
        // publishers to PACmod
        ros::Publisher steer_cmd_pub_;
        ros::Publisher turn_cmd_pub_;

        float wheelAngle_to_steeringAngle(float wheelAngle);

        float curvature_to_wheelAngle(float curv);

        void PublishSteering(void);

        void PublishTurn(void);
};


#endif