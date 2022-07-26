//
//  NeedleInsertionRobot.h
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/4/22.
//

#pragma once

#include <cmath>
#include <map>

#include "GalilController.h"


#define DEFAULT_GALIL_IP "192.168.1.201"
#define ROBOT_NUM_AXES 4

// NULL AXES operations
const float NULL_FLOAT_AXIS = std::numeric_limits<float>::lowest();
inline static bool isNullAxis(float axis){ return axis == NULL_FLOAT_AXIS; }

struct RobotLimit
{
    constexpr static const float MIN = std::numeric_limits<float>::lowest(), 
                                 MAX = std::numeric_limits<float>::max();
    bool active = true; // whether limit is active or not
    float min = MIN, max = MAX;
    float position = 0.0;

    /** Functions */
    RobotLimit(){}
    RobotLimit(bool active) : active(active) {}
    RobotLimit(float min, float max) : RobotLimit( min, max, true, 0.0 ) {}
    RobotLimit(float min, float max, float position) : RobotLimit( min, max, true, position ) {}
    RobotLimit(float min, float max, bool active, float position) : min(min), max(max), active(active), position(position) {}
    RobotLimit(const RobotLimit& limit) = default;

    inline float limit(const float& position, bool absolute=true) const
    {
        if (absolute) {
            float limited_pos = std::min( std::max( this->min, position ), this->max );
            setPosition(limited_pos);
            float result = this->adjustPos(limited_pos);
            
            return  result;    
        } else {
            float limited_pos = std::min( std::max( this->min, position+this->position ), this->max )
            setPosition(limited_pos);
            float result = this->adjustPos(limited_pos);

            return this->adjustPos( limited_pos );
        }

    } // limit

    float getPosition() {
        return this->position;
    }

    private:
        inline float adjustPos(const float& position) const
        {
            return position - this->position;
        } //adjustPos
        
        void setPosition(const float& position) {
            this->position = position;
        }

}; // struct: RobotLimit

class NeedleInsertionRobot
{
public:
    // constructor and destructor
    /** Default constructor with pre-defined galil IP address.
            
     */
    NeedleInsertionRobot() : NeedleInsertionRobot(DEFAULT_GALIL_IP) {}
   /** Constructor with galil controller IP address
    
    @param ipAddress GStringIn of the IP address of the controller.
  
    */
    NeedleInsertionRobot(GCStringIn ipAddress);
    ~NeedleInsertionRobot();
    
    /** Get the galil controller for custom commands
            
            @returns GalilController pointer to current controller
     */
    const std::shared_ptr<GalilController> getGalilController() {return m_galilController;}
    /** Send a direct commadn to the galiil controller
            @param command GCStringIn of the galil command to send
     */
    GCStringOut galilCommand(GCStringIn command) const { return m_galilController->command(command); }
    GCStringOut galilCommand(std::string& command) const { return m_galilController->command(command); }
    
    /** Motor control commands */
    void abort(){m_galilController->abort();}
    
    /** Motor Control shortcuts */
    void allMotorsOn () { bool axes[ROBOT_NUM_AXES] = {true, true, true, true}; motorsOn (axes); }
    void allMotorsOff() { bool axes[ROBOT_NUM_AXES] = {true, true, true, true}; motorsOff(axes); }
    
    /** Get whether the motors are on or not */
    const bool* getAxesMoving();
    const bool  getAxisMoving(const size_t axis) { return getAxesMoving()[axis]; }
    const std::array<RobotLimit, ROBOT_NUM_AXES>& getLimits() { return m_limits; }
    const RobotLimit& getLimit(const size_t axis) {return getLimits()[axis]; }
    const bool* getMotorsOn() const { return m_activeAxes; }

    /** Get motor position commands
        @param axes boolean array of which axes that would like to be queried
        @param absolute (default=false) whether to use absolute positioning or not
     */
    float* getPosition(const bool axes[ROBOT_NUM_AXES], const bool absolute=false) const;
    // single axis-implementations
    float getPositionX (const bool absolute=false) const {bool axes[ROBOT_NUM_AXES] = {true,  false, false, false}; return getPosition(axes, absolute)[0]; }
    float getPositionY (const bool absolute=false) const {bool axes[ROBOT_NUM_AXES] = {false, true,  false, false}; return getPosition(axes, absolute)[1]; }
    float getPositionZ (const bool absolute=false) const {bool axes[ROBOT_NUM_AXES] = {false, false, true,  false}; return getPosition(axes, absolute)[2]; }
    float getPositionLS(const bool absolute=false) const {bool axes[ROBOT_NUM_AXES] = {false, false, false, true};  return getPosition(axes, absolute)[3]; }
    
    // Assert motion is complete
    void motionComplete(){ m_galilController->motionComplete(); }
    // bool useMotionComplete(bool motionComplete) {m_useMotionComplete = motionComplete; return m_useMotionComplete; }

    // turn on/off the motors
    /** 
     * If axes[index] == true, then it will perform the motors(On/Off) on that axis. Otherwise, no operation will be performed
     */
    void motorsOn (const bool axes[ROBOT_NUM_AXES]);
    void motorsOff(const bool axes[ROBOT_NUM_AXES]);
    
    /**
     Move the axes
     
     @param axes (long[GALIL_NUM_AXES]) array of axes move counts to move
     @param absolute (bool, Default=false) whether to perform absolute movements or not
     
     */
    void moveAxes(const float axes[ROBOT_NUM_AXES], bool absolute=false) const {if (absolute) return moveAxesAbsolute(axes); else return moveAxesRelative(axes);}
    void moveAxesAbsolute(const float axes[ROBOT_NUM_AXES]) const ; // absolute move axes
    void moveAxesRelative(const float axes[ROBOT_NUM_AXES]) const ; // relative move axes
    
    /* Controller commands to set control parameters */
    /** Set PID constants
     
     */
    void setPID_P(const long kp_axes[ROBOT_NUM_AXES]) const;
    void setPID_I(const long ki_axes[ROBOT_NUM_AXES]) const;
    void setPID_D(const long kd_axes[ROBOT_NUM_AXES]) const;
    
    
    /** Set speed control variables  */
    void setAcceleration(const float ac_axes[ROBOT_NUM_AXES]) const;
    void setDeceleration(const float dc_axes[ROBOT_NUM_AXES]) const;
    void setSpeed(const float sp_axes[ROBOT_NUM_AXES]) const;

    /** Robot Limit functionality */
    /** Set whether a limit is active or not 
     * @return boolean of active status for the current limit
    */
    std::array<bool, ROBOT_NUM_AXES> setLimitsActive ( std::array<bool, ROBOT_NUM_AXES> actives );
    std::array<bool, ROBOT_NUM_AXES> setLimitsActive ( std::vector< std::pair<size_t, bool> > actives );
    bool setLimitActiveX ( bool active ) { return setLimitsActive( { std::make_pair( 0, active ) } )[0]; }
    bool setLimitActiveY ( bool active ) { return setLimitsActive( { std::make_pair( 1, active ) } )[1]; }
    bool setLimitActiveZ ( bool active ) { return setLimitsActive( { std::make_pair( 2, active ) } )[2]; }
    bool setLimitActiveLS( bool active ) { return setLimitsActive( { std::make_pair( 3, active ) } )[3]; }

    /** Set the robot limits using the RobotLimit struct */
    void setLimits ( std::array<RobotLimit, ROBOT_NUM_AXES> limits ) { m_limits = limits; } // wipes everything clean
    void setLimits ( std::initializer_list< std::pair<size_t, RobotLimit> > limits ); // for updating limits
    void setLimits ( std::map<size_t, RobotLimit> limits ); // for updating limits
    void setLimitX ( RobotLimit limit ){ setLimits( { { {0, limit} } } ); }
    void setLimitY ( RobotLimit limit ){ setLimits( { { {1, limit} } } ); }
    void setLimitZ ( RobotLimit limit ){ setLimits( { { {2, limit} } } ); }
    void setLimitLS( RobotLimit limit ){ setLimits( { { {3, limit} } } ); }
    
    /** Stop moving axes
     @param axes bool[GALIL_NUM_AXES[] on whether to stop particular axes or not
     */
    void stopAxes(const bool axes[ROBOT_NUM_AXES]) const;
    void stopAllAxes() const { stopAxes(s_axes); }
    
    /** Zero specific axes
     @param axes bool[GALIL_NUM_AXES[] on whether to stop particular axes or not
     */
    void zeroAxes(const bool axes[ROBOT_NUM_AXES]) const;
    void zeroAllAxes() const { zeroAxes(s_axes); }

// public member functions
public: // static defaults
    /* Speed defaults */
    constexpr static const float s_default_speed[ROBOT_NUM_AXES]        = {2.5, 2.5, 2.5, 2.0}; // speeds per mm
    constexpr static const float s_default_acceleration[ROBOT_NUM_AXES] = {1.5, 1.5, 1.5, 10000.0/43680.0};
    constexpr static const float s_default_deceleration[ROBOT_NUM_AXES] = {1.5, 1.5, 1.5, 10000.0/43680.0};
    constexpr static const float s_countsPerDistance[ROBOT_NUM_AXES] = {2000.0, 2000.0, 2000.0, 43680.0}; // calibrated counts/mm
    
    /* PID defaults*/
    constexpr static const long s_default_kP[ROBOT_NUM_AXES] = { 54,  15,  54,  25};
    constexpr static const long s_default_kI[ROBOT_NUM_AXES] = {  4,   4,   4,   4};
    constexpr static const long s_default_kD[ROBOT_NUM_AXES] = {480, 332, 480, 480};

    /* Axes defaults */
    constexpr static const bool s_axes[GALIL_NUM_AXES] = {false, true, true, true, true}; // Galil axes that are turned on
    
// public static members
    
private: // private members
    std::shared_ptr<GalilController> m_galilController;
    bool m_activeAxes[ROBOT_NUM_AXES] = {false, false, false, false};
    bool m_useMotionComplete = false; // unused
    std::array<RobotLimit, ROBOT_NUM_AXES> m_limits;
    

protected: // protected members
    /* Index for each axes in the Galil Controller Axes */
    const static size_t m_xIdx  = 1; // B: x axis
    const static size_t m_yIdx  = 2; // C: y axis
    const static size_t m_zIdx  = 3; // D: z axis
    const static size_t m_lsIdx = 4; // E: linear stage axis
    

protected: // protected fnctions
    /** Robot Limit Handling 
     * @arg absolute: whether the current axes command is an absolute positioning or relative
     * 
     * @returns array of floats with size [ROBOT_NUM_AXES] with limited motion
    */
    float* limitMotion( const float axes[ROBOT_NUM_AXES], bool absolute ) const;

    /** Galil Controller Interfacing */
    size_t getGalilAxisIndex(size_t axis) const
    {
        switch (axis)
        {
            case 0:
                return m_xIdx;
                
            case 1:
                return m_yIdx;
                
            case 2:
                return m_zIdx;
                
            case 3:
                return m_lsIdx;
                
            default:
                return -1;
        }
    } // getGalilAxisIndex
    
    /* Robot <---> Galil Axis mappings */
    template <typename T>
    T* galilToRobotAxes(const T axes[GALIL_NUM_AXES]) const
    {
        T* robot_axes = new T[ROBOT_NUM_AXES];
        
        // copy over data
        for (size_t i = 0; i < ROBOT_NUM_AXES; i++)
            robot_axes[i] = axes[getGalilAxisIndex(i)];
        
        return robot_axes;
        
    } // galilToRobotAxes
    
    bool* robotToGalilAxes(const bool axes[ROBOT_NUM_AXES]) const
    {
        bool* gc_axes = new bool[GALIL_NUM_AXES];
        
        // initialize gc_axes
        for (int i = 0; i < GALIL_NUM_AXES; i++)
            gc_axes[i] = false;
        
        // remap the axis index
        for (int i = 0; i < ROBOT_NUM_AXES; i++)
            gc_axes[getGalilAxisIndex(i)] = axes[i];
        
        return gc_axes;
        
    } // robotToGalilAxes (bool)
    
    long* robotToGalilAxes(const long axes[ROBOT_NUM_AXES]) const
    {
        long* gc_axes = new long[GALIL_NUM_AXES];
        
        // initialize gc_axes
        for (int i = 0; i < GALIL_NUM_AXES; i++)
            gc_axes[i] = NULL_LONG_AXIS;
        
        // remap the axis index
        for (int i = 0; i < ROBOT_NUM_AXES; i++)
            gc_axes[getGalilAxisIndex(i)] = axes[i];
        
        return gc_axes;
        
    } // robotToGalilAxes (long)
    
    /* Conversions */
    /** Convert encoder counts to distance
        @param axes const long array axes of the distance measurements to counts
     
        @returns float array of the measurements in encoder counts
     
     */
    float* countsToDistance(const long axes[ROBOT_NUM_AXES]) const
    {
        float* f_axes = new float[ROBOT_NUM_AXES];
        
        for (int i = 0; i < ROBOT_NUM_AXES; i++)
            f_axes[i] = isNullAxis(axes[i]) ? NULL_FLOAT_AXIS : ((float) axes[i])/s_countsPerDistance[i];
        
        return f_axes;
        
    } // float
    
    /** Convert distance to encoder counts
        @param axes const float array axes of the distance measurements to counts
     
        @returns long array of the measurements in encoder counts
     */
    long* distanceToCounts(const float axes[ROBOT_NUM_AXES]) const
    {
        long* l_axes = new long[ROBOT_NUM_AXES];
        
        // perform the conversion while checking for null axis
        for (int i = 0; i < ROBOT_NUM_AXES; i++)
            l_axes[i] = isNullAxis(axes[i]) ? NULL_LONG_AXIS : std::round(axes[i] * s_countsPerDistance[i]);
        
        return l_axes;
        
    } // distanceToCounts
    
// protected
    
}; // class: NeedleInsertionRobot
