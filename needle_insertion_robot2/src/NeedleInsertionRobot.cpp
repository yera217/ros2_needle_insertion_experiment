//
//  NeedleInsertionRobot.cpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/4/22.
//

#include "galil/NeedleInsertionRobot.h"
#include <iostream>

/* Static Value References */
const float NeedleInsertionRobot::s_default_speed[ROBOT_NUM_AXES];
const float NeedleInsertionRobot::s_default_acceleration[ROBOT_NUM_AXES];
const float NeedleInsertionRobot::s_default_deceleration[ROBOT_NUM_AXES];
const long NeedleInsertionRobot::s_default_kP[ROBOT_NUM_AXES];
const long NeedleInsertionRobot::s_default_kI[ROBOT_NUM_AXES];
const long NeedleInsertionRobot::s_default_kD[ROBOT_NUM_AXES];
const bool NeedleInsertionRobot::s_axes[GALIL_NUM_AXES];
const float NeedleInsertionRobot::s_countsPerDistance[ROBOT_NUM_AXES];


NeedleInsertionRobot::NeedleInsertionRobot(GCStringIn ipAddress) : m_galilController(std::make_shared<GalilController>(ipAddress))
{
    // turn off all motors
    allMotorsOff();
    
    // Set speed controls
    setSpeed(s_default_speed);
    setAcceleration(s_default_acceleration);
    setDeceleration(s_default_deceleration);
    
    // set PID controls
    setPID_P(s_default_kP);
    setPID_I(s_default_kI);
    setPID_D(s_default_kD);

    // set limits
    std::array<RobotLimit, ROBOT_NUM_AXES> limits;
    for (int i = 0; i < limits.size(); i++)
        limits[i] = RobotLimit(false); // empty robot limit non-active

    // for
    setLimits(limits); 

} // constructor with IP address

NeedleInsertionRobot::~NeedleInsertionRobot()
{
    m_galilController.reset(); // delete pointer
    
} // destructor

const bool* NeedleInsertionRobot::getAxesMoving()
{
    bool* gc_axesMoving = m_galilController->getAxesMoving();

    bool* axesMoving = galilToRobotAxes( gc_axesMoving );

    return axesMoving;

} // NeedleInsertionRobot::getAxesMoving

float* NeedleInsertionRobot::getPosition(const bool axes[ROBOT_NUM_AXES], const bool absolute) const
{
    bool* gc_axes = robotToGalilAxes(axes);
    long* gc_counts = m_galilController->getPosition(gc_axes, absolute); // get encoder counts

    long* counts = galilToRobotAxes(gc_counts); // convert to Robot axes mappings
    float* positions = countsToDistance(counts); // convert encoder counts to robot positions
    
    return positions;
    
    
} // NeedleInsertionRobot::getPosition

float* NeedleInsertionRobot::limitMotion( const float axes[ROBOT_NUM_AXES], bool absolute ) const
{
    float* axes_limited = new float[ROBOT_NUM_AXES];

    // get positioning
    for (int i = 0; i < ROBOT_NUM_AXES; i++) // limit the motion
         axes_limited[i] = m_limits[i].active && !isNullAxis(axes[i]) ? 
                            m_limits[i].limit( axes[i], absolute ) : 
                            axes[i];
    
    return axes_limited;

} // NeedleInsertionRobot::limitMotion

void NeedleInsertionRobot::motorsOn(const bool axes[ROBOT_NUM_AXES])
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_galilController->motorsOn(gc_axes);

    // copy over the motor axes
    for(int i = 0; i < ROBOT_NUM_AXES; i++)
        if (axes[i])
            m_activeAxes[i] = true;
    
} // NeedleInsertionRobot::motorsOn

void NeedleInsertionRobot::motorsOff(const bool axes[ROBOT_NUM_AXES])
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_galilController->motorsOff(gc_axes);

    // copy over the motor axes
    for(int i = 0; i < ROBOT_NUM_AXES; i++)
        if (axes[i])
            m_activeAxes[i] = false;
    
} // NeedleInsertionRobot::motorsOff


/* movement commands */
void NeedleInsertionRobot::moveAxesAbsolute(const float axes[ROBOT_NUM_AXES]) const
{
    // convert distance measurements to counts
    float* axes_limited = limitMotion( axes, true ); // limit the motion
    long* counts_axes = distanceToCounts( axes_limited );
    
    // remove any axes that don't move
    for(int i = 0; i < ROBOT_NUM_AXES; i++)
        if (!m_activeAxes[i]) // if axis is not turned on
            counts_axes[i] = NULL_LONG_AXIS; // null the axis

    // convert to galil controller axes
    long* gc_axes = robotToGalilAxes(counts_axes);
    
    // send the command
    m_galilController->moveAxesAbsolute(gc_axes);
    
    
} // NeedleInsertionRobot::moveAxesAbsolute

void NeedleInsertionRobot::moveAxesRelative(const float axes[ROBOT_NUM_AXES]) const
{
    // convert distance measurements to counts
    float* axes_limited = limitMotion( axes, false ); // limit the motion
    long* counts_axes = distanceToCounts( axes_limited );

    // remove any axes that don't move
    for(int i = 0; i < ROBOT_NUM_AXES; i++)
        if (!m_activeAxes[i]) // if axis is not turned on
            counts_axes[i] = NULL_LONG_AXIS; // null the axis
    
    // convert to galil controller axes
    long* gc_axes = robotToGalilAxes(counts_axes);
    
    // send the command
    m_galilController->moveAxesRelative(gc_axes);
    
} // NeedleInsertionRobot::moveAxesRelative

/* set PID commands */
void NeedleInsertionRobot::setPID_P(const long kp_axes[ROBOT_NUM_AXES]) const
{
    long* gc_axes = robotToGalilAxes(kp_axes);
    
    m_galilController->setPID_P(gc_axes);
    
    
} // NeedleInsertionRobot::setPID_P

void NeedleInsertionRobot::setPID_I(const long ki_axes[ROBOT_NUM_AXES]) const
{
    long* gc_axes = robotToGalilAxes(ki_axes);
    
    m_galilController->setPID_I(gc_axes);
    
    
} // NeedleInsertionRobot::setPID_I

void NeedleInsertionRobot::setPID_D(const long kd_axes[ROBOT_NUM_AXES]) const
{
    long* gc_axes = robotToGalilAxes(kd_axes);
    
    m_galilController->setPID_D(gc_axes);
    
    
} // NeedleInsertionRobot::setPID_D


/* set speed commands */
void NeedleInsertionRobot::setAcceleration(const float ac_axes[ROBOT_NUM_AXES]) const
{
    long* l_ac_axes = distanceToCounts(ac_axes); // convert to encoder counts
    
    long* gc_axes = robotToGalilAxes(l_ac_axes); // convert to galil axes format
    
    m_galilController->setAcceleration(gc_axes);
    
} //NeedleInsertionRobot::setAcceleration

void NeedleInsertionRobot::setDeceleration(const float dc_axes[ROBOT_NUM_AXES]) const
{
    long* l_dc_axes = distanceToCounts(dc_axes); // convert to encoder counts
    
    long* gc_axes = robotToGalilAxes(l_dc_axes); // convert to galil axes format
    
    m_galilController->setDeceleration(gc_axes);
    
} //NeedleInsertionRobot::setDeceleration

void NeedleInsertionRobot::setSpeed(const float sp_axes[ROBOT_NUM_AXES]) const
{
    long* l_sp_axes = distanceToCounts(sp_axes); // convert to encoder counts
    
    long* gc_axes = robotToGalilAxes(l_sp_axes); // convert to galil axes format
    
    m_galilController->setSpeed(gc_axes);
    
} //NeedleInsertionRobot::setSpeed

std::array<bool, ROBOT_NUM_AXES> NeedleInsertionRobot::setLimitsActive ( std::array<bool, ROBOT_NUM_AXES> actives )
{
    std::array<bool, ROBOT_NUM_AXES> current_actives;

    // update the current limit actives
    for(int i = 0; i < m_limits.size(); i++)
        m_limits[i].active = actives[i];

    // set the current active return values
    for(int i = 0; i < m_limits.size(); i++)
        current_actives[i] = m_limits[i].active;

    return current_actives;
} // NeedleInsetionRobot::setLimitsActive

std::array<bool, ROBOT_NUM_AXES> NeedleInsertionRobot::setLimitsActive ( std::vector< std::pair<size_t, bool> > actives )
{
    std::array<bool, ROBOT_NUM_AXES> current_actives;

    for (auto kv : actives)
    {
        // unpack the pair
        size_t index = kv.first;
        bool active  = kv.second;

        // ensure no out-of-bounds
        if (0 <= index && index < m_limits.size())
            m_limits[index].active = active;

    } // for

    // set the current active return values
    for(int i = 0; i < m_limits.size(); i++)
        current_actives[i] = m_limits[i].active;

    return current_actives;
} // NeedleInsetionRobot::setLimitsActive

void NeedleInsertionRobot::setLimits( std::initializer_list< std::pair<size_t, RobotLimit> > limits)
{
    for (const auto& kv : limits)
    {
        // unpack the pair
        const size_t& index     = kv.first;
        const RobotLimit& limit = kv.second;

        // ensure safe-indexing
        if (0 <= index && index < m_limits.size())
            m_limits[index] = limit;

    } // for

} // NeedleInsertionRobot::setLimits

void NeedleInsertionRobot::setLimits( std::map<size_t, RobotLimit> limits)
{
    for (const auto& kv : limits)
    {
        // unpack the pair
        const size_t& index     = kv.first;
        const RobotLimit& limit = kv.second;

        // ensure safe-indexing
        if (0 <= index && index < m_limits.size())
            m_limits[index] = limit;

    } // for

} // NeedleInsertionRobot::setLimits

/* Axes commands */
void NeedleInsertionRobot::stopAxes(const bool axes[ROBOT_NUM_AXES]) const
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_galilController->stopAxes(gc_axes);
    
} // NeedleInsertionRobot::stopAxes


void NeedleInsertionRobot::zeroAxes(const bool axes[ROBOT_NUM_AXES]) const
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_galilController->zeroAxes(gc_axes);
    
} // NeedleInsertionRobot::zeroAxes




