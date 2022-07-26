#include <cstdio>
#include <memory>

// rclcpp
#include "rclcpp/rclcpp.hpp"
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
// #include "galil/NeedleInsertionRobot.h"
#include "needle_insertion_robot/insertion_robot_node.hpp"

// /* Static Value References */
// const float NeedleInsertionRobot::s_default_speed[ROBOT_NUM_AXES];
// const float NeedleInsertionRobot::s_default_acceleration[ROBOT_NUM_AXES];
// const float NeedleInsertionRobot::s_default_deceleration[ROBOT_NUM_AXES];
// const long NeedleInsertionRobot::s_default_kP[ROBOT_NUM_AXES];
// const long NeedleInsertionRobot::s_default_kI[ROBOT_NUM_AXES];
// const long NeedleInsertionRobot::s_default_kD[ROBOT_NUM_AXES];
// const bool NeedleInsertionRobot::s_axes[GALIL_NUM_AXES];
// const float NeedleInsertionRobot::s_countsPerDistance[ROBOT_NUM_AXES];

// helpful macros
#define ROBOT_BIND_AXIS_CMD_CB(msg_t, x, axis, absolute) (std::function<void(const msg_t)>) std::bind(&NeedleInsertionRobotNode::x, this, axis, std::placeholders::_1, absolute)
#define ROBOT_BIND_FN(x) std::bind(&NeedleInsertionRobotNode::x, this, std::placeholders::_1)
#define ROBOT_BIND_PUBLISHER(x) std::bind(&NeedleInsertionRobotNode::x, this)
#define ROBOT_BIND_SERVICE(x) std::bind(&NeedleInsertionRobotNode::x, this, std::placeholders::_1, std::placeholders::_2)
#define ROBOT_BIND_AXIS_SERVICE(srv_t, x, axis) (std::function <void (const srv_t::Request::SharedPtr, const srv_t::Response::SharedPtr)>) \
                                                 std::bind(&NeedleInsertionRobotNode::x, this, axis, std::placeholders::_1, std::placeholders::_2)

using namespace std::chrono_literals;

NeedleInsertionRobotNode::NeedleInsertionRobotNode(const std::string& name)
    : Node(name)
{
    // declare parameters
    std::string ip_address = this->declare_parameter("robot.ip_address", DEFAULT_GALIL_IP); // IP address of the robot
    
    // initialize the robot
    RCLCPP_INFO(this->get_logger(), "Connecting to Robot at IP Address: %s", ip_address.c_str());
    m_robot = std::make_shared<NeedleInsertionRobot>(ip_address.c_str());
    RCLCPP_INFO(this->get_logger(), "Connection established.");
    m_robot->allMotorsOff();
    
    // parameters
    float x_speed   = this->declare_parameter( "axis.x.speed",        NeedleInsertionRobot::s_default_speed[0] );
    float x_accel   = this->declare_parameter( "axis.x.acceleration", NeedleInsertionRobot::s_default_acceleration[0] );
    float x_decel   = this->declare_parameter( "axis.x.deceleration", NeedleInsertionRobot::s_default_deceleration[0] );
    long  x_kp      = this->declare_parameter( "axis.x.kp",           NeedleInsertionRobot::s_default_kP[0] );
    long  x_ki      = this->declare_parameter( "axis.x.ki",           NeedleInsertionRobot::s_default_kI[0] );
    long  x_kd      = this->declare_parameter( "axis.x.kd",           NeedleInsertionRobot::s_default_kD[0] );
    bool  x_lim_act = this->declare_parameter( "axis.x.limit.active", false );
    float x_lim_min = this->declare_parameter( "axis.x.limit.min",    RobotLimit::MIN );
    float x_lim_max = this->declare_parameter( "axis.x.limit.max",    RobotLimit::MAX );

    float y_speed   = this->declare_parameter( "axis.y.speed",        NeedleInsertionRobot::s_default_speed[1] );
    float y_accel   = this->declare_parameter( "axis.y.acceleration", NeedleInsertionRobot::s_default_acceleration[1] );
    float y_decel   = this->declare_parameter( "axis.y.deceleration", NeedleInsertionRobot::s_default_deceleration[1] );
    long  y_kp      = this->declare_parameter( "axis.y.kp",           NeedleInsertionRobot::s_default_kP[1] );
    long  y_ki      = this->declare_parameter( "axis.y.ki",           NeedleInsertionRobot::s_default_kI[1] );
    long  y_kd      = this->declare_parameter( "axis.y.kd",           NeedleInsertionRobot::s_default_kD[1] );
    bool  y_lim_act = this->declare_parameter( "axis.y.limit.active", false );
    float y_lim_min = this->declare_parameter( "axis.y.limit.min",    RobotLimit::MIN );
    float y_lim_max = this->declare_parameter( "axis.y.limit.max",    RobotLimit::MAX );

    float z_speed   = this->declare_parameter( "axis.z.speed",        NeedleInsertionRobot::s_default_speed[2] );
    float z_accel   = this->declare_parameter( "axis.z.acceleration", NeedleInsertionRobot::s_default_acceleration[2] );
    float z_decel   = this->declare_parameter( "axis.z.deceleration", NeedleInsertionRobot::s_default_deceleration[2] );
    long  z_kp      = this->declare_parameter( "axis.z.kp",           NeedleInsertionRobot::s_default_kP[2] );
    long  z_ki      = this->declare_parameter( "axis.z.ki",           NeedleInsertionRobot::s_default_kI[2] );
    long  z_kd      = this->declare_parameter( "axis.z.kd",           NeedleInsertionRobot::s_default_kD[2] );
    bool  z_lim_act = this->declare_parameter( "axis.z.limit.active", false );
    float z_lim_min = this->declare_parameter( "axis.z.limit.min",    RobotLimit::MIN );
    float z_lim_max = this->declare_parameter( "axis.z.limit.max",    RobotLimit::MAX );

    float ls_speed   = this->declare_parameter( "axis.linear_stage.speed",        NeedleInsertionRobot::s_default_speed[3] );
    float ls_accel   = this->declare_parameter( "axis.linear_stage.acceleration", NeedleInsertionRobot::s_default_acceleration[3] );
    float ls_decel   = this->declare_parameter( "axis.linear_stage.deceleration", NeedleInsertionRobot::s_default_deceleration[3] );
    long  ls_kp      = this->declare_parameter( "axis.linear_stage.kp",           NeedleInsertionRobot::s_default_kP[3] );
    long  ls_ki      = this->declare_parameter( "axis.linear_stage.ki",           NeedleInsertionRobot::s_default_kI[3] );
    long  ls_kd      = this->declare_parameter( "axis.linear_stage.kd",           NeedleInsertionRobot::s_default_kD[3] );
    bool  ls_lim_act = this->declare_parameter( "axis.linear_stage.limit.active", false );
    float ls_lim_min = this->declare_parameter( "axis.linear_stage.limit.min",    RobotLimit::MIN );
    float ls_lim_max = this->declare_parameter( "axis.linear_stage.limit.max",    RobotLimit::MAX );

    float speed[] = { x_speed, y_speed, z_speed, ls_speed };
    float accel[] = { x_accel, y_accel, z_accel, ls_accel };
    float decel[] = { x_decel, y_decel, z_decel, ls_decel };
    long     kp[] = { x_kp   , y_kp   , z_kp   , ls_kp    };
    long     ki[] = { x_ki   , y_ki   , z_ki   , ls_ki    };
    long     kd[] = { x_kd   , y_kd   , z_kd   , ls_kd    };
    
    update_robotParams( speed, accel, decel, kp, ki, kd); // set Robot control parameters
    m_robot->setLimits({
        { 0, { x_lim_min,  x_lim_max,  x_lim_act  } }, 
        { 1, { y_lim_min,  y_lim_max,  y_lim_act  } },
        { 2, { z_lim_min,  z_lim_max,  z_lim_act  } },
        { 3, { ls_lim_min, ls_lim_max, ls_lim_act } }
    });  // set the robot limits

    m_paramCallbackHandle = this->add_on_set_parameters_callback( ROBOT_BIND_FN(service_onSetParamsCallback) ); 

    // initalize subscribers
    m_sub_AxCommandX  = this->create_subscription<AxisMsg_t>("axis/command/x",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 0, false));
    m_sub_AxCommandY  = this->create_subscription<AxisMsg_t>("axis/command/y",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 1, false));
    m_sub_AxCommandZ  = this->create_subscription<AxisMsg_t>("axis/command/z",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 2, false));
    m_sub_AxCommandLS = this->create_subscription<AxisMsg_t>("axis/command/linear_stage", 1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 3, false));

    m_sub_AxAbsCommandX  = this->create_subscription<AxisMsg_t>("axis/command/absolute/x",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 0, true));
    m_sub_AxAbsCommandY  = this->create_subscription<AxisMsg_t>("axis/command/absolute/y",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 1, true));
    m_sub_AxAbsCommandZ  = this->create_subscription<AxisMsg_t>("axis/command/absolute/z",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 2, true));
    m_sub_AxAbsCommandLS = this->create_subscription<AxisMsg_t>("axis/command/absolute/linear_stage", 1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 3, true));

    m_sub_AxRelCommandX  = this->create_subscription<AxisMsg_t>("axis/command/relative/x",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 0, false));
    m_sub_AxRelCommandY  = this->create_subscription<AxisMsg_t>("axis/command/relative/y",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 1, false));
    m_sub_AxRelCommandZ  = this->create_subscription<AxisMsg_t>("axis/command/relative/z",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 2, false));
    m_sub_AxRelCommandLS = this->create_subscription<AxisMsg_t>("axis/command/relative/linear_stage", 1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 3, false));
    
    // initalize publishers
    m_pub_AxPosX  = this->create_publisher<AxisMsg_t>("axis/position/x",            10);
    m_pub_AxPosY  = this->create_publisher<AxisMsg_t>("axis/position/y",            10);
    m_pub_AxPosZ  = this->create_publisher<AxisMsg_t>("axis/position/z",            10);
    m_pub_AxPosLS = this->create_publisher<AxisMsg_t>("axis/position/linear_stage", 10);

    m_pub_AxMovingX  = this->create_publisher<Bool>("axis/state/moving/x",            10);
    m_pub_AxMovingY  = this->create_publisher<Bool>("axis/state/moving/y",            10);
    m_pub_AxMovingZ  = this->create_publisher<Bool>("axis/state/moving/z",            10);
    m_pub_AxMovingLS = this->create_publisher<Bool>("axis/state/moving/linear_stage", 10);

    m_pub_AxStateX  = this->create_publisher<Bool>("axis/state/on/x",            10);
    m_pub_AxStateY  = this->create_publisher<Bool>("axis/state/on/y",            10);
    m_pub_AxStateZ  = this->create_publisher<Bool>("axis/state/on/z",            10);
    m_pub_AxStateLS = this->create_publisher<Bool>("axis/state/on/linear_stage", 10);

    // initialize services
    m_srv_abort         = this->create_service<Trigger>("abort",                          ROBOT_BIND_SERVICE(service_abort));
    m_srv_toggleAxisX   = this->create_service<Trigger>("axis/state/toggle/x",            ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 0));
    m_srv_toggleAxisY   = this->create_service<Trigger>("axis/state/toggle/y",            ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 1));
    m_srv_toggleAxisZ   = this->create_service<Trigger>("axis/state/toggle/z",            ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 2));
    m_srv_toggleAxisLS  = this->create_service<Trigger>("axis/state/toggle/linear_stage", ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 3));

    m_srv_zeroAxisX     = this->create_service<Trigger>("axis/zero/x",              ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   0));
    m_srv_zeroAxisY     = this->create_service<Trigger>("axis/zero/y",              ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   1));
    m_srv_zeroAxisZ     = this->create_service<Trigger>("axis/zero/z",              ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   2));
    m_srv_zeroAxisLS    = this->create_service<Trigger>("axis/zero/linear_stage",   ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   3));

    // create timers
    m_positionTimer  = this->create_wall_timer( 10ms, ROBOT_BIND_PUBLISHER(publish_CurrentPosition) );
    m_stateTimer     = this->create_wall_timer( 20ms, ROBOT_BIND_PUBLISHER(publish_CurrentState) );
    
    RCLCPP_INFO(this->get_logger(), "Robot initialized and ready for operation.");
    
} // Constructor
NeedleInsertionRobotNode::~NeedleInsertionRobotNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down robot node: '%s'", this->get_name());
    
m_robot->allMotorsOff();
} // destructor

/* Service callbacks */
void NeedleInsertionRobotNode::service_abort(const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
{
    RCLCPP_WARN(this->get_logger(), "Abort command triggered!");
    m_robot->abort();
    RCLCPP_WARN(this->get_logger(), "Aborted!");

    res->success = true;

} // service_abort

rcl_interfaces::msg::SetParametersResult NeedleInsertionRobotNode::service_onSetParamsCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    // setup of return message
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // initialize arrays
    float speed[] = {NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS};
    float accel[] = {NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS};
    float decel[] = {NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS};

    long kp[] = {NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS};
    long ki[] = {NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS};
    long kd[] = {NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS};
    std::map<size_t, RobotLimit> limits;

    // TODO: need to add functionality for changing speed and gain params adaptively
    for (const auto& param : parameters )
    {
        RCLCPP_INFO( this->get_logger(), "Parameter update name is: %s", param.get_name().c_str() );
        // axis: x
        if(param.get_name().compare("axis.x.speed") == 0)
            speed[0] = param.as_double();
        
        else if(param.get_name().compare("axis.x.acceleration") == 0)
            accel[0] = param.as_double();

        else if(param.get_name().compare("axis.x.deceleration") == 0)
            decel[0] = param.as_double();

        else if(param.get_name().compare("axis.x.kp") == 0)
            kp[0] = param.as_int();

        else if(param.get_name().compare("axis.x.ki") == 0)
            ki[0] = param.as_int();
            
        else if(param.get_name().compare("axis.x.kd") == 0)
            kd[0] = param.as_int();

        else if(param.get_name().compare("axis.x.limit.active") == 0)
        {
            if (limits.find(0) == limits.end())
                limits[0] = RobotLimit(m_robot->getLimit(0));

            limits[0].active = param.as_bool();

        } // else if

        else if(param.get_name().compare("axis.x.limit.min") == 0)
        {
            if (limits.find(0) == limits.end())
                limits[0] = RobotLimit(m_robot->getLimit(0));

            limits[0].min = param.as_double();

        } // else if

        else if(param.get_name().compare("axis.x.limit.max") == 0)
        {
            if (limits.find(0) == limits.end())
                limits[0] = RobotLimit(m_robot->getLimit(0));

            limits[0].max = param.as_double();

        } // else if

        // axis: y
        else if(param.get_name().compare("axis.y.speed") == 0)
            speed[1] = param.as_double();
        
        else if(param.get_name().compare("axis.y.acceleration") == 0)
            accel[1] = param.as_double();

        else if(param.get_name().compare("axis.y.deceleration") == 0)
            decel[1] = param.as_double();

        else if(param.get_name().compare("axis.y.kp") == 0)
            kp[1] = param.as_int();

        else if(param.get_name().compare("axis.y.ki") == 0)
            ki[1] = param.as_int();

        else if(param.get_name().compare("axis.y.kd") == 0)
            kd[1] = param.as_int();

        else if(param.get_name().compare("axis.y.limit.active") == 0)
        {
            if (limits.find(1) == limits.end())
                limits[1] = RobotLimit(m_robot->getLimit(1));
                
            limits[1].active = param.as_bool();

        } // else if

        else if(param.get_name().compare("axis.y.limit.min") == 0)
        {
            if (limits.find(1) == limits.end())
                limits[1] = RobotLimit(m_robot->getLimit(1));
                
            limits[1].min = param.as_double();

        } // else if

        else if(param.get_name().compare("axis.y.limit.max") == 0)
        {
            if (limits.find(1) == limits.end())
                limits[1] = RobotLimit(m_robot->getLimit(1));
                
            limits[1].max = param.as_double();

        } // else if

        // axis: z
        else if(param.get_name().compare("axis.z.speed") == 0)
            speed[2] = param.as_double();
        
        else if(param.get_name().compare("axis.z.acceleration") == 0)
            accel[2] = param.as_double();

        else if(param.get_name().compare("axis.z.limit.active") == 0)
        {
            if (limits.find(2) == limits.end())
                limits[2] = RobotLimit(m_robot->getLimit(2));
                
            limits[2].active = param.as_bool();

        } // else if

        else if(param.get_name().compare("axis.z.limit.min") == 0)
        {
            if (limits.find(2) == limits.end())
                limits[2] = RobotLimit(m_robot->getLimit(2));
                
            limits[2].min = param.as_double();

        } // else if

        else if(param.get_name().compare("axis.z.limit.max") == 0)
        {
            if (limits.find(2) == limits.end())
                limits[2] = RobotLimit(m_robot->getLimit(2));
                
            limits[2].max = param.as_double();

        } // else if
            
        // axis: linear_stage
        else if(param.get_name().compare("axis.linear_stage.speed") == 0)
            speed[3] = param.as_double();
        
        else if(param.get_name().compare("axis.linear_stage.acceleration") == 0)
            accel[3] = param.as_double();
            
        else if(param.get_name().compare("axis.linear_stage.deceleration") == 0)
            decel[3] = param.as_double();
            
        else if(param.get_name().compare("axis.linear_stage.kp") == 0)
            kp[3] = param.as_int();
            
        else if(param.get_name().compare("axis.linear_stage.ki") == 0)
            ki[3] = param.as_int();
            
        else if(param.get_name().compare("axis.linear_stage.kd") == 0)
            kd[3] = param.as_int();

        else if(param.get_name().compare("axis.linear_stage.limit.active") == 0)
        {
            if (limits.find(3) == limits.end())
                limits[3] = RobotLimit(m_robot->getLimit(3));
                
            limits[3].active = param.as_bool();

        } // else if

        else if(param.get_name().compare("axis.linear_stage.limit.min") == 0)
        {
            if (limits.find(3) == limits.end())
                limits[3] = RobotLimit(m_robot->getLimit(3));
                
            limits[3].min = param.as_double();

        } // else if

        else if(param.get_name().compare("axis.linear_stage.limit.max") == 0)
        {
            if (limits.find(3) == limits.end())
                limits[3] = RobotLimit(m_robot->getLimit(3));
                
            limits[3].max = param.as_double();

        } // else if
        
        else
            result.successful = false;

    } // for

    // update the robot parameters
    if (result.successful)
    {
        update_robotParams(speed, accel, decel, kp, ki, kd); 
        update_robotLimits(limits);
        RCLCPP_INFO(this->get_logger(), "Limit map size: %d", limits.size());

    } // if

    return result;


} // service_onSetParamsCallback

void NeedleInsertionRobotNode::service_toggleAxis(int axis, const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
{
    bool command_positions[ROBOT_NUM_AXES] = { false, false, false, false };
    std::string axisName;

    switch(axis)
    {
        case 0: // axis X
            axisName = "X";
            break;
            
        case 1: // axis Y
            axisName = "Y";
            break;
            
        case 2: // axis Z
            axisName = "Z";
            break;
            
        case 3: // axis LS
            axisName = "LS";
            break;
            
        default:
            res->success = false;
            return;
            
    } // switch

    command_positions[axis] = true;
    const char message_fmt[] = "Axis %s has been turned %s.";
    char message[100];
    if (m_robot->getMotorsOn()[axis])
    {
        RCLCPP_INFO(this->get_logger(), "Toggling axis %s off", axisName.c_str());
        m_robot->motorsOff(command_positions);
        sprintf( message, message_fmt, axisName.c_str(), "off" );
    
    } // if
    else
    {
        RCLCPP_INFO(this->get_logger(), "Toggling axis %s on", axisName.c_str());
        m_robot->motorsOn(command_positions);
        sprintf( message, message_fmt, axisName.c_str(), "on" );

    } // else
    
    // set result success
    res->success = true;
    res->message = message;

} // service_toggleAxis

void NeedleInsertionRobotNode::service_zeroAxis(int axis, const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
{
    bool command_axes[ROBOT_NUM_AXES] = { false, false, false, false };
    std::string axisName;
    
    switch(axis)
    {
        case 0: // axis X
            axisName = "X";
            break;
            
        case 1: // axis Y
            axisName = "Y";
            break;
            
        case 2: // axis Z
            axisName = "Z";
            break;
            
        case 3: // axis LS
            axisName = "LS";
            break;
            
        default:
            res->success = false;
            return;
            
    } // switch

    command_axes[axis] = true;

    try
    {
        RCLCPP_INFO(this->get_logger(), "Zeroing axis %s.", axisName.c_str());
        m_robot->zeroAxes( command_axes );
        res->success = true;
    }
    catch (GReturn e)
    { 
        char message[100];
        sprintf(message, "Error zeroing axis %s. Error Code %d", axisName.c_str(), e);
        RCLCPP_ERROR(this->get_logger(), message);
        res->success = false;
        res->message = message;
    }

    

} // service_zeroAxis


/* Subscriber callbacks */
void NeedleInsertionRobotNode::topic_callbackAxisCommand(int axis, const AxisMsg_t::SharedPtr msg, bool absolute)
{
    std::string axisName;
    
    // perform command to specified axis
    float command_positions[ROBOT_NUM_AXES] = {NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS};
    switch(axis)
    {
        case 0: // axis X
            axisName = "X";
            break;
            
        case 1: // axis Y
            axisName = "Y";
            break;
            
        case 2: // axis Z
            axisName = "Z";
            break;
            
        case 3: // axis LS
            axisName = "LS";
            break;
            
        default:
            return;
            
    } // switch
    
    float cmd_pos = msg->data;
    command_positions[axis] = cmd_pos;
    try
    {
        RCLCPP_INFO(this->get_logger(), "Commanding axis '%s' for %.2f mm", axisName.c_str(), cmd_pos);
        if (m_robot->getMotorsOn()[axis])
            m_robot->moveAxes(command_positions, absolute);

        else
            RCLCPP_WARN(this->get_logger(), "Axis '%s' is not on! Turn the axis on before moving.", axisName.c_str());

        // m_robot->motionComplete(); // ensure motion is finished
        
    } // try
    catch( GReturn ec )
    {
        RCLCPP_WARN(this->get_logger(), "Moving axis throwing error code: %d! You may be publishing a command before motion has finished.", ec);
        return;

    } // catch
    
} // topic_callbackAxisCommand

/* Publisher functions */
void NeedleInsertionRobotNode::publish_CurrentPosition() // publishes the position of the axes
{
    auto msg_x  = AxisMsg_t();
    auto msg_y  = AxisMsg_t();
    auto msg_z  = AxisMsg_t();
    auto msg_ls = AxisMsg_t();

    // TODO: sample robot coordinates
    float* positions;
    try
    {
        positions = m_robot->getPosition(m_robotAxes.data(), true);
    
    } // try
    catch(GReturn ec)
    {
        RCLCPP_WARN(this->get_logger(), "Error getting positions: error code = %d. Robot maybe buffered.", ec);
        return;
    
    } // catch
    catch(std::invalid_argument e)
    {
        RCLCPP_WARN(this->get_logger(), "Error getting positions: error = '%s'. Robot maybe buffered.", e.what());
        return;

    } // catch: invalid_argument

    RCLCPP_DEBUG(this->get_logger(), "Position: %.2f, %.2f, %.2f, %.2f", positions[0], positions[1], positions[2], positions[3]);

    // set the position data
    msg_x.data  = positions[0];
    msg_y.data  = positions[1];
    msg_z.data  = positions[2];
    msg_ls.data = positions[3];

    // publish
    m_pub_AxPosX  -> publish( msg_x );
    m_pub_AxPosY  -> publish( msg_y );
    m_pub_AxPosZ  -> publish( msg_z );
    m_pub_AxPosLS -> publish( msg_ls );
    
} // publish_CurrentState

void NeedleInsertionRobotNode::publish_CurrentState() // publishes the state of the axis
{
    // get the current axis states
    const bool* axes_on;
    try
    {
        axes_on = m_robot->getMotorsOn();
    }
    catch (GReturn ec)
    {
        RCLCPP_ERROR(this->get_logger(), "Error getting which motors are on! Galil Error code = %d!", ec);
        return;

    } // catch
    
    const bool* axes_moving;
    try
    {
        axes_moving = m_robot->getAxesMoving();
    }
    catch (GReturn ec)
    {
        RCLCPP_ERROR(this->get_logger(), "Error getting which motors are moving! Galil Error code = %d!", ec);
        return;
        
    } // catch

    // setup the messages
    auto msg_on_x      = Bool();
    auto msg_on_y      = Bool();
    auto msg_on_z      = Bool();
    auto msg_on_ls     = Bool();

    auto msg_moving_x  = Bool();
    auto msg_moving_y  = Bool();
    auto msg_moving_z  = Bool();
    auto msg_moving_ls = Bool();

    // set the data for the messages
    msg_on_x.data      = axes_on[0];
    msg_on_y.data      = axes_on[1];
    msg_on_z.data      = axes_on[2];
    msg_on_ls.data     = axes_on[3];

    msg_moving_x.data  = axes_moving[0];
    msg_moving_y.data  = axes_moving[1];
    msg_moving_z.data  = axes_moving[2];
    msg_moving_ls.data = axes_moving[3];

    // publish the messages
    m_pub_AxStateX   -> publish( msg_on_x );
    m_pub_AxStateY   -> publish( msg_on_y );
    m_pub_AxStateZ   -> publish( msg_on_z );
    m_pub_AxStateLS  -> publish( msg_on_ls ); 

    m_pub_AxMovingX  -> publish( msg_moving_x );
    m_pub_AxMovingY  -> publish( msg_moving_y );
    m_pub_AxMovingZ  -> publish( msg_moving_z );
    m_pub_AxMovingLS -> publish( msg_moving_ls ); 

} // publish_CurrentState

void NeedleInsertionRobotNode::update_robotLimits(const std::map<size_t, RobotLimit>& limits)
{
    m_robot->setLimits(limits);

    // iterate through to get the name
    for (const auto& kv : limits)
    {
        const size_t& axis = kv.first;
        const char* axis_name;
        switch(axis)
        {
            case 0:
                axis_name = "X'";
                break;

            case 1:
                axis_name = "Y'";
                break;

            case 2:
                axis_name = "Z'";
                break;

            case 3:
                axis_name = "LS'";
                break;

            default:
                RCLCPP_WARN(this->get_logger(), "Update Limit: Invalid axis: %d", axis);
                continue;
            
        } // switch

        const RobotLimit& limit = m_robot->getLimit(axis);

        RCLCPP_INFO( this->get_logger(), "Updated limits of axis '%s' to: Active=%d, Min=%.3f, Max=%.3f",
                    axis_name, limit.active, limit.min, limit.max );

    } // for

} // update_robotLimits

void NeedleInsertionRobotNode::update_robotParams(float speed[ROBOT_NUM_AXES],
                        float accel[ROBOT_NUM_AXES],
                        float decel[ROBOT_NUM_AXES],
                        long     kp[ROBOT_NUM_AXES],
                        long     ki[ROBOT_NUM_AXES],
                        long     kd[ROBOT_NUM_AXES])
{
    m_robot->setSpeed(speed);
    m_robot->setAcceleration(accel);
    m_robot->setDeceleration(decel);
    
    m_robot->setPID_P(kp);
    m_robot->setPID_I(ki);
    m_robot->setPID_D(kd);

    RCLCPP_INFO(this->get_logger(), "Updating Robot configuration parameters.");

} // update_robotParams

/* =================== END OF CLASS IMPLEMENTATION ====================== */
/* =================== MAIN METHOD ====================================== */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<NeedleInsertionRobotNode>());
    rclcpp::shutdown();
    
    return 0;
    
} // main
