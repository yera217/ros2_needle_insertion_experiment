#pragma once

#include <array>

// rclcpp
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"

// ROS messages
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"

// ROS services
#include "std_srvs/srv/trigger.hpp"

// custom headers
#include "galil/NeedleInsertionRobot.h"

const float RobotLimit::MIN, RobotLimit::MAX;

class NeedleInsertionRobotNode : public rclcpp::Node
{
public:
    // helpful type definitions
    typedef std_msgs::msg::Float32 AxisMsg_t;
    typedef std_srvs::srv::Trigger Trigger;
    typedef std_msgs::msg::Bool Bool;
    
    // constructors
    NeedleInsertionRobotNode(const std::string& name = "NeedleInsertionRobot");
    virtual ~NeedleInsertionRobotNode();

private: //members
    std::shared_ptr<NeedleInsertionRobot> m_robot;

    const std::array<bool, 4> m_robotAxes = {true, true, true, true};     // Robot axes to use (use them all)
    
    // timers
    rclcpp::TimerBase::SharedPtr m_positionTimer, m_stateTimer;

    // subscribers
    rclcpp::Subscription<AxisMsg_t>::SharedPtr m_sub_AxCommandX, 
                                               m_sub_AxCommandY,
                                               m_sub_AxCommandZ, 
                                               m_sub_AxCommandLS,

                                               m_sub_AxAbsCommandX, 
                                               m_sub_AxAbsCommandY,
                                               m_sub_AxAbsCommandZ, 
                                               m_sub_AxAbsCommandLS,
                                               
                                               m_sub_AxRelCommandX,
                                               m_sub_AxRelCommandY,
                                               m_sub_AxRelCommandZ,
                                               m_sub_AxRelCommandLS;
    
    // publishers
    rclcpp::Publisher<AxisMsg_t>::SharedPtr m_pub_AxPosX, 
                                            m_pub_AxPosY, 
                                            m_pub_AxPosZ, 
                                            m_pub_AxPosLS;
    
    rclcpp::Publisher<Bool>::SharedPtr m_pub_AxStateX,
                                       m_pub_AxStateY,
                                       m_pub_AxStateZ,
                                       m_pub_AxStateLS,
                                       
                                       m_pub_AxMovingX,
                                       m_pub_AxMovingY,
                                       m_pub_AxMovingZ,
                                       m_pub_AxMovingLS;

    // services
    rclcpp::Service<Trigger>::SharedPtr m_srv_abort,
                                                       
                                        m_srv_toggleAxisX,
                                        m_srv_toggleAxisY,
                                        m_srv_toggleAxisZ,
                                        m_srv_toggleAxisLS,
                                        
                                        m_srv_zeroAxisX,
                                        m_srv_zeroAxisY,
                                        m_srv_zeroAxisZ,
                                        m_srv_zeroAxisLS;

    // callbacks
    OnSetParametersCallbackHandle::SharedPtr m_paramCallbackHandle;

private: // methods
    // services
    void service_abort(const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res);
    rcl_interfaces::msg::SetParametersResult service_onSetParamsCallback(const std::vector<rclcpp::Parameter>& parameters);
    void service_toggleAxis(int axis, const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res);
    void service_zeroAxis(int axis, const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res);

    // subscriber callbacks
    void topic_callbackAxisCommand(int axis, const AxisMsg_t::SharedPtr msg, bool absolute);

    // publisher functions
    void publish_CurrentPosition(); // publishes the position of the axes    
    void publish_CurrentState(); // publishes the state of the axis

    // Robot functions
    inline bool axisMoving(size_t axis){ return m_robot->getAxisMoving(axis); }
    void update_robotLimits(const std::map<size_t, RobotLimit>& limits);
    void update_robotParams(float speed[ROBOT_NUM_AXES],
                            float accel[ROBOT_NUM_AXES],
                            float decel[ROBOT_NUM_AXES],
                            long     kp[ROBOT_NUM_AXES],
                            long     ki[ROBOT_NUM_AXES],
                            long     kd[ROBOT_NUM_AXES]);
    
}; // class: NeedleInsertionRobotNode