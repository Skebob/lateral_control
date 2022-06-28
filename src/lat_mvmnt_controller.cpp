#include "lat_mvmnt_controller.hpp"
#include "ros/ros.h"

#define PI 3.14159265

LateralControl::LateralControl(ros::NodeHandle* nh){
    // subsribers from SSC inferface
    steer_mode_sub_ = nh->subscribe("ssc/arbitrated_steering_commands", 1000, &LateralControl::callback_steering, this);
    // publishers to PACmod
    steer_cmd_pub_ = nh->advertise<pacmod_msgs::PositionWithSpeed>("as_rx/steer_cmd", 1000);
    turn_cmd_pub_ = nh->advertise<pacmod_msgs::PacmodCmd>("as_rx/turn_cmd", 1000);
}

/*
@param void
@return void
*/
void LateralControl::PublishSteering(void){
    pacmod_msgs::PositionWithSpeed cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.angular_position = steeringAngle_;
    cmd.angular_velocity_limit = curvature_limit_; // not sure if this is correct, may need to be a fuction of current car velocity
    steer_cmd_pub_.publish(cmd);
}

/*
@param void
@return void
*/
void LateralControl::PublishTurn(void){
    pacmod_msgs::PacmodCmd steering_cmd;
}


/*
@param wheelAngle (rad): desired wheel deflection angle, this value is calculated from curvature_to_wheelAngle()
@return steeringAngle (rad): returns desired position of steering angle, will make sure steeringAngle is whithin interval [-10.5, 10.5]
*/
float LateralControl::wheelAngle_to_steeringAngle(float wheelAngle){
    float steeringAngle = 0;
    float steeringAngle_deg = 0;
    
    float wheelAngle_deg = wheelAngle * 180 / PI;

    steeringAngle_deg = wheelAngle_deg * 19.2; // from experimentally collected data, assumes steering ratio of 19.1:1

    steeringAngle = steeringAngle_deg * PI / 180;

    if(steeringAngle > 10.5) steeringAngle = 10.5;

    if(steeringAngle < -10.5) steeringAngle = -10.5;

    return steeringAngle;
}

/*
@param curv (1/m): desired curvature, this value comes from SSC_interface
@return wheelAngle (rad): deflection of wheels
*/
float LateralControl::curvature_to_wheelAngle(float curv){
    float wheelAngle = 0;
    /*
        BICYCLE MODEL 
            wheel_angle = arctan([wheelbase] / [radius])
            radius = 1 / curvature 
            =>  wheel angle = arctan([wheelbase] * [curvature])
    */
   
    wheelAngle = atan(curv*wheelbase);

    return wheelAngle;
} 

/* 
    @breif  called when desired curvature is published from ssc inferface
            [from ssc_interface] -> curvature ---[bicycle model]---> wheel angle ---[experimental formula]---> steering angle -> [to PACmod]
 */
void LateralControl::callback_steering(const automotive_platform_msgs::SteerMode::ConstPtr& msg){
    if(msg->mode == 1){
        // autonomy is active
        curvature_ = msg->curvature;
        curvature_limit_ = msg->max_curvature_rate;

        wheelAngle_ = curvature_to_wheelAngle(curvature_);

        steeringAngle_ = wheelAngle_to_steeringAngle(wheelAngle_);
        
        PublishSteering();
    }
}