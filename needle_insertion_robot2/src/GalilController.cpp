#include "galil/GalilController.h"

#include <algorithm>


GalilController::GalilController(GCStringIn ipAddress)
{
    try
    {
        e(GOpen(ipAddress, &m_gc)); // open the connection
    } // try
    catch (std::exception e)
    {
        GALIL_OUT("Galil connection Error at " << ipAddress);
        throw e;
        
    } // catch    
} // GalilController Constructor

GalilController::~GalilController()
{
    GClose(m_gc);
    
} // GalilController destructor

GCStringOut GalilController::bufferToGCStringOut(char* buffer, unsigned int buffer_size)
{
    GCStringOut stringout = new char[GALIL_BUFFER_SIZE];
    
    memcpy(stringout, buffer, buffer_size);
    
    return stringout;
    
} // GalilController::bufferToGCStringOut

GCStringOut GalilController::command(GCStringIn command) 
{
    GALIL_OUT("Galil Command: " << command); // display galil commands
    // GCmdT(m_gc, command, m_buffer, GALIL_BUFFER_SIZE, NULL); // no error checking trimmed version
    e(GCmdT(m_gc, command, m_buffer, GALIL_BUFFER_SIZE, NULL)); // trimmed version
    // this->e(GCommand(m_gc, command, m_buffer, G_SMALL_BUFFER, &m_bytesRead)); // full version
    
    GCStringOut response = bufferToGCStringOut(m_buffer, GALIL_BUFFER_SIZE); // get the response
    GALIL_OUT("Galil Response: " << response); // display galil response messages
    
    flushBuffer(); // flush the buffer
    
    return response;
    
} // GalilController::command

GCStringOut GalilController::motionComplete()
{
    GCStringOut response = command("MC");
    return response;  // TODO: implement
    
} // GalilController:: motionComplete

GReturn GalilController::motorsOn(const bool axes[GALIL_NUM_AXES])
{
    bool any_on = false;
    std::string command = "SH ";
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        any_on = any_on || axes[i];
        if (axes[i])
            command += axisName(i);
        
    } // for
    
    if (any_on)
        this->command( command );
    
    return 0;
    
} // GalilController::motorsOn

GReturn GalilController::motorsOff(const bool axes[GALIL_NUM_AXES])
{
    bool any_on = false;
    std::string command = "MO ";
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        any_on = any_on || axes[i];
        if (axes[i])
            command += axisName(i);
        
    } // for
    
    if (any_on)
        this->command( command );
    
    return 0;
    
} // GalilController::motorsOff

GReturn GalilController::moveAxes(const long axes[GALIL_NUM_AXES], bool absolute)
{
    if (absolute)
        return moveAxesAbsolute(axes);
    
    else
        return moveAxesRelative(axes);
    
} // GalilController::moveAxes

GReturn GalilController::moveAxesAbsolute(const long axes[GALIL_NUM_AXES])
{
    // format the command message
    std::string command = "PA " + commaSeparateValues(std::vector<long>(axes, axes + GALIL_NUM_AXES));
    std::string bg_command = "BG ";
    bool any_axes_on = false;
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        //        command += std::to_string(axes[i]) + (i < GALIL_NUM_AXES - 1 ? ",": "");
        if (isNullAxis(axes[i]))
            continue;
        
        else
            any_axes_on = true;
        
        bg_command += axisName(i); // TODO: need to change to detect absolute changes
        
    } // for
    
    // check if any axes were moved
    if (any_axes_on)
    {
        this->command(command); // send the move command
        this->command(bg_command); // send the begin command
    } // if

    return 0;
    
} // GalilController::moveAxesAbsolute

GReturn GalilController::moveAxesRelative(const long axes[GALIL_NUM_AXES])
{
    // format the command message
    std::string mv_command = "PR " + commaSeparateValues(std::vector<long>(axes, axes + GALIL_NUM_AXES));
    std::string bg_command = "BG ";
    bool any_axes_on = false;
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        //        mv_command += std::to_string(axes[i]) + (i < GALIL_NUM_AXES - 1 ? ",": "");
        if (axes[i] == 0 || isNullAxis(axes[i]))
            continue;
        
        else
        {
            any_axes_on = true;
            bg_command += axisName(i);

        } // else
    } // for

    // check if any axes were moved
    if (any_axes_on)
    {
        this->command(mv_command); // send the move command
        this->command(bg_command); // send the begin command
    } // if
    
    return 0;
    
} // GalilController::moveAxesRelative

bool* GalilController::getAxesMoving()
{
    std::string command = "MG ";
    // sample command "MG _BGA, _BGB, _BGC, _BGD, _BGE";

    // Set up the galil command
    for (int axis = 0; axis < GALIL_NUM_AXES; axis++)
    {
        command += std::string("_BG") + axisName(axis);
        if (axis < GALIL_NUM_AXES - 1) // check for last axis
            command += ", \",\", "; // add comma-separator

    } // for
    
    // get the response
    GCStringOut response = this->command(command);
    std::string s_response(response);
    
    // setup return
    bool* axesRunning = new bool[GALIL_NUM_AXES];
    for (size_t axis = 0; axis < GALIL_NUM_AXES, axis++;)
        axesRunning[axis] = false;
    
    // parse the response
    std::string token;
    s_response += ","; // add-on to end to get last part
    std::size_t pos = 0;

    int counter = 0;
    
    while((pos = s_response.find(",")) != std::string::npos && (counter < GALIL_NUM_AXES))
    {
        token = s_response.substr(0, pos);
        bool isnumeric = token.find_first_not_of("-0123456789.") == std::string::npos;
        
        try
        {
            // add to result
            if (counter >= GALIL_NUM_AXES)
                break;
        

            axesRunning[counter++] = std::stof(token) > 0;
            
        } // try
        catch (std::invalid_argument e)
        {
            continue; // not a float
        }

        s_response.erase(0, pos + 1); // remove the processed part of string

    } // while

    return axesRunning;

} // GalilController::getAxesMoving

long* GalilController::getPosition(const bool axes[GALIL_NUM_AXES], bool absolute)
{
    // prepare the command
    std::string command = "";
    if (absolute)
        command = "TP ";
    else
        command = "PR ";
    
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        if (absolute)
        {
            if (i < GALIL_NUM_AXES && axes[i])
                command += axisName(i);
        }
        else
        {
            if (i < GALIL_NUM_AXES - 1 )
                command += axes[i] ? "?," : ",";
            
            else // end of list
                command += axes[i] ? "?" : "";
        } // else
    } // for
    
    // get the response
    GCStringOut response = this->command(command);
    std::string s_response(response);
    
    // parse the response
    long* positions = new long[GALIL_NUM_AXES];
    for (int i = 0; i < GALIL_NUM_AXES; i++)
        positions[i] = NULL_LONG_AXIS; // initalize array

    std::string token;
    s_response += ","; // add-on to end to get last part
    std::size_t pos = 0;

    int counter = -1;
    auto update_counter = [&counter, &axes](){
        while(!axes[++counter] && counter < GALIL_NUM_AXES) 
            continue;
        };
    update_counter();
    
    while((pos = s_response.find(",")) != std::string::npos)
    {
        // std::cout << "GalilContoller::getPosition: s_response = " << s_response <<  " | counter = " << counter << std::endl;
        token = s_response.substr(0, pos);
        bool isnumeric = token.find_first_not_of("-0123456789 ") == std::string::npos;

        if (isnumeric)
        {
            // add to result
            if (counter >= GALIL_NUM_AXES)
                break;
        

            positions[counter] = std::stol(token);
            update_counter();
            
        } // if

        s_response.erase(0, pos + 1); // remove the processed part of string
        

    } // while

    // std::cout << "GalilController::getPositions return = " << positions[0]
    //                                                 << "," << positions[1]
    //                                                 << "," << positions[2]
    //                                                 << "," << positions[3]
    //                                                 << "," << positions[4] << std::endl;

    return positions;

    
    
} // GalilController::getPosition


GReturn GalilController::setPID_P(const long kp_axes[GALIL_NUM_AXES])
{
    std::string command = "KP " + commaSeparateValues(std::vector<long>(kp_axes, kp_axes + GALIL_NUM_AXES));
    
    this->command(command);
    
    return 0;
    
} // GalilController::setPID_P


GReturn GalilController::setPID_I(const long ki_axes[GALIL_NUM_AXES])
{
    std::string command = "KI " + commaSeparateValues(std::vector<long>(ki_axes, ki_axes + GALIL_NUM_AXES));
    
    this->command(command);
    
    return 0;
    
} // GalilController::setPID_I


GReturn GalilController::setPID_D(const long kd_axes[GALIL_NUM_AXES])
{
    std::string command = "KD " + commaSeparateValues(std::vector<long>(kd_axes, kd_axes + GALIL_NUM_AXES));
    
    this->command(command);
    
    return 0;
    
} // GalilController::setPID_D


GReturn GalilController::setAcceleration(const long ac_axes[GALIL_NUM_AXES])
{
    std::string command = "AC " + commaSeparateValues(std::vector<long>(ac_axes, ac_axes + GALIL_NUM_AXES));
    
    this->command(command); // send update command
    
    return 0;
    
} // GalilController::setAcceleration


GReturn GalilController::setDeceleration(const long dc_axes[GALIL_NUM_AXES])
{
    std::string command = "DC " + commaSeparateValues(std::vector<long>(dc_axes, dc_axes + GALIL_NUM_AXES));
    
    this->command(command); // send update command
    
    return 0;
    
} // GalilController::setDeceleration


GReturn GalilController::setSpeed(const long sp_axes[GALIL_NUM_AXES])
{
    std::string command = "SP " + commaSeparateValues(std::vector<long>(sp_axes, sp_axes + GALIL_NUM_AXES));
    
    this->command(command); // send update command
    
    return 0;
    
} // GalilController::setSpeed


GReturn GalilController::stopAxes(const bool axes[GALIL_NUM_AXES])
{
    std::string command = "ST ";
    
    for (int i = 0; i < GALIL_NUM_AXES; i++)
        command += axes[i] ? std::string(1, axisName(i)) : "";
    
    this->command(command); // send the command
    
    return 0;
    
} // GalilController::stopAxes


GReturn GalilController::zeroAxes(const bool axes[GALIL_NUM_AXES])
{
    std::string command = "DP ";

    // setup the commmand
    for (int i = 0; i < GALIL_NUM_AXES; i++)
        command += axes[i] ? "0, " : ", ";

    this->command(command); // send the command
    
    return 0;
    
} // GalilController::zeroAxes

