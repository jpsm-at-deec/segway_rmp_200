#ifndef _SEGWAYRMP200_DRIVER_H
#define _SEGWAYRMP200_DRIVER_H

#include "ftdimodule.h"
#include "ftdiserver.h"
#include "mutex.h"
#include "math.h"
#include <string>
#include <vector>
#include <queue>

/**
 * \brief structure of a USB packet 
 *
 * This new data type has all the information of a data packet send to or received
 * from the USB communication device. This data type is only 18 unsigned char values 
 * arranged as a vector.
 *
 */
typedef struct 
{
  /**
   * \brief packet data
   *
   * This vector has all the data of a packet that has been received or that is going
   * to be sent. The format of this packet is as follows:
   *
   */  
  unsigned char data[18];
}segway_packet;

/**
 * \brief Valid gain schedules for the segway
 *
 * This new data type enumerates all the possible gain schedules of a segway platform.
 * This data type is only used on the balancing mode and ignored in the tractor mode.
 * The possible gain schedule values are:
 *
 * - light: for relativelly small weights (20 kg) placed near the platform.
 * - tall: for relativelly small weights (20 kg) distributed along the vertical axis
 *         of the robot.
 * - heavy: for heavy weights (40 kg) placed near the platform.
 *
 */
typedef enum {light,tall,heavy} gain;

/**
 * \brief Valid operation modes of the segway
 *
 * This new data type enumerates all the possible operation modes of a segway platform.
 * The possible operation modes are:
 *
 * - tractor: in this mode the balancing feature is disabled and the platform needs an
 *            additional support point (castor wheel). In this mode it is possible to
 *            command the robot to move forward/reverse and also turn.
 * - balance: in this mode the segway balances itself, trying to keep the given position.
 *            In this mode it is possible to move the robot without a castor wheel.
 * - power down: the platform is shut down and no power is provided to the motors. It is 
 *               possible to enter this mode from either the tractor and balance modes.
 *
 */
typedef enum {tractor=1,balance=2,power_down=3} op_mode;

/**
 * \brief structure of segway status
 *
 * This datatype holds the values of internal information retrieved from the
 * hardware platform. It tries to give a general overview of what's happening
 * in the platform in a given moment.
 */
struct TSegwayRMP200Status 
{
    // Translational velocities
    float right_wheel_velocity;
    float left_wheel_velocity;
    // Angular rates and angles
    float pitch_angle;
    float pitch_rate;
    float roll_angle;
    float roll_rate;
    float yaw_rate;
    // Displacements
    float left_wheel_displ;
    float right_wheel_displ;
    float forward_displ;
    float yaw_displ;
    // Other configurations
    float servo_frames;
    float left_torque;
    float right_torque;
    float ui_battery;
    float powerbase_battery;
    op_mode operation_mode;
    op_mode hardware_mode;
    gain gain_schedule;
};


/**
 * \brief Segway RMP 200 driver
 *
 * This class implements the basic interface to communcate with an RMP200 segway platform,
 * configure it and also send motion commands. By default the communication with the 
 * hardware platform is achieved through USB.
 *
 * At construction time the object tries to connect to a platform. If only one platform is
 * connected to the computer, the default constructor can be used, but if there exist 
 * multiple segway platforms, it is necessary to pass the serial number to the constructor
 * In order to connect to the desired one and avoid errors.
 * 
 * When the object is properly contructed, the two main threads of the class start. These 
 * threads are:
 *
 * - The command thread: which is responsible of sending the mnotion command to the platform
 *                       at regular intervals. Currently the update rate for the motion 
 *                       command is fixed to 50 times per second. 
 *
 *                       When the user gives a motion command using the move() or stop() 
 *                       functions, the new command is not immediatelly send to the robot,
 *                       but stored into internal parameters of the class. It is this thread
 *                       that sends the new commands to the robot at regular intervals. The
 *                       same happens with all the configuration parameters, when the 
 *                       corresponding function is called, the command is internally stored
 *                       and send in the next command packet to the platform.
 *
 *                       At the moment it is not possible to change the update rate of the 
 *                       motion commands.
 *
 * - The feedback thread: this thread is responsible of reding all the data sent by the 
 *                        robot platform and store it in the internal parameters of the 
 *                        class as fast as they are sent.
 *
 *                        When the user tries to read the value of one of the parameters 
 *                        of the platform, the returned value is the one stored inside 
 *                        the class, which are updated by this thread.
 *
 *                        At the moment only data messages from 2 to 7 are handled, as well
 *                        as the heart beat. The other messages are received but ignored.
 *
 * The public interface of this class alows to completelly configure all the parameters of
 * the segway platform as well as monitor each of the available feedback parameters. Also
 * there exist an operator << to show all feedback information already formated.
 *
 * This class also has support for unexpected hardware events such as diconnection of the
 * USB cable, turning the platform power off or no reception of the heartbeat message for
 * a long period. Primary, this actions are reported by dedicated events that are created 
 * internally at construction time. These events are:
 *
 * - cable_disconnected_event: when activated, it notifies that the USB cable has been
 *                             disconnected from the platform. When this happens, the normal
 *                             operation of the class is interrupted, but it does not crash.
 *                             After this event is received, it will be necessary to call the
 *                             close() function and the connect() function again to restore
 *                             normal operation.
 *
 *                             When the connection is restored, this event is automatically
 *                             cleared.
 *
 * - power_off_event : when activated, it notifies that the platform is not powerd on. This
 *                     normally happens at start up when the green button on the platform is
 *                     not lid or when an error ocurrs. When this event is active, the normal
 *                     operation of the class is interrupted, but it is not necessary to
 *                     close and reconnect the object. When the problem is solved, the class
 *                     automatically returns to the normal operation mode.
 * 
 *                     When the connection is restored, this event is automatically cleared.
 * 
 * - no_heartbeat_event : when activated, it notifies that the platform is turned on, but no
 *                        data is received from it. The class has an internal thread that
 *                        works as a watchdog: when a heartbeat message is received, the 
 *                        watchdog counter is reset, but if this message is not received for
 *                        a long time, the watchdog times out and generates an event.
 * 
 *                        When this event is active, the normal operation of the class is 
 *                        interrupted, but it is not necessary to close and reconnect the
 *                        object. When the problem is solved, the class automatically returns
 *                        to the normal operation. When the connection is restored, this event 
 *                        is automatically cleared.
 *
 * Finally, in addition to the previously defined events, the class also provide an event to
 * notify when a new complete status data is available. This event can be used to avoid
 * reading the same data several times. 
 */
class CSegwayRMP200
{
  public:
    static const float COUNTSDEG_2_RAD           = M_PI/(7.8f*180.f);
    static const float COUNTSDEG_2_RADSEC        = M_PI/(7.8f*180.f);
    static const float COUNTS_2_METSEC           = 1.f/332.f;
    static const float SECFRAMES_2_FRAMESEC      = 1.f/0.01f;
    static const float COUNTSMETR_2_METR         = 1.f/33215.f;
    static const float COUNTSREV_2_RADS          = 2.f*M_PI/112644.f;
    static const float COUNTSNEWTMETR_2_NEWTMETR = 1.f/1094.f;
    static const float COUNTS_2_VOLT             = 0.0125f;
    static const float COUNTS_2_VOLT_OFFSET      = 1.4f;
    static const float COUNTSVOLT_2_VOLT         = 1.f/4.f;
    static const float COUNTSMPH_2_METRSEC       = 3.6f*147.f/1.609344f;
    static const float COUNTS_2_RADSEC           = 1024.f;
    static const float RADS_2_DEGS               = 180.f/M_PI;
    
    /**
     * \brief segway PID
     *
     * This constant holds the PID number used by the segway platforms. It is defined
     * as static so it can be used even if no instance of this class exists. 
     */
    static const short int pid;
    /**
     * \brief Segway platform description
     *
     * This constant holds the description string of the segway platforms. It is
     * defined as static so it can be used even if no instance of this class
     * exists. 
     *
     * This description can be used in the FTDI server to identify the segway platforms
     * from any other FTDI device connected to the computer.
     */
    static const std::string description;
  private:
    /**
     * \brief unique identifier of the segway platform
     *
     * This string has the unique identifier of the segway platform. This string is
     * initialized at construction time and can not be modified afterwards. The 
     * get_segway_id() function can be used to get this string. 
     *
     * This string is also used to create unique identifier for all the threads and
     * events used inside the class. This identifier is build from the serial number
     * of the segway platform to which it is connected.
     *
     */
    std::string id;
    /**
     * \brief mutex for the status data
     *
     * This mutex is used to control the access to the status variables (shared memory)
     * of the class. These variables are periodically updated by the feedback thread 
     * and they can be read by the user at any time using the get functions. 
     *
     * This mutex is used by the feedback thread and all the get status function to 
     * avoid data corruption while simultaneously reading and writing to the status 
     * variable of the segway platform. There exist a different mutex for the command 
     * data.
     *
     */
    CMutex access_status;
    /**
     * \brief mutex for the command data
     *
     * This mutex is used to control the access to the command variables (shared memory)
     * of the class. These variables are changed by the user at any time using the mov()
     * or stop() functions, and the new command is actually send to the segway platform
     * periodically by the command thread. 
     *
     * This mutex is used by the command thread and all the motion function to avoid data
     * corruption while simultaneously reading and writing to the command variable of the
     * segway platform. There exist a different mutex for the status data.
     */
    CMutex access_command;
    /**
     * \brief a reference to the FTDI USB device
     *
     * This attribute points to the communication device used to send and receive data 
     * to and from the segway platform. The communication device is cretaed inside the 
     * object at initialization time using the CFTDIServer and the description or serial
     * number of the desired platform via the connect() function.
     *
     * It is not possible to send or receive data to or from the platform until the 
     * driver is not connected to the hardware platform. Any attempt to do so will 
     * result in an exception being thrown.
     *
     */
    CFTDI *comm_dev;
    /**
     * \brief reference to the unique ftdi_server
     *
     * This reference to the unique ftdi server is initialized when an object of
     * this class is first created. It is used to create and handle all the
     * FTDI device drivers used to access the segway platform. The object pointed 
     * by this reference is shared by all objects in any application.
     *
     */
    CFTDIServer *ftdi_server; 
    /**
     * \brief Reference to the unique event handler
     *
     * This reference to the unique event handler is initialized when an object of
     * this class is first created. It is used to create and handle all the
     * segway platform events. The object pointed by this reference is shared by all
     * objects in any application.
     */
    CEventServer *event_server;
    /**
     * \brief Reference to the unique thread handler
     *
     * This reference to the unique thread handler is initialized when an object of
     * this class is first created. It is used to create and handle all the
     * segway platform threads. The object pointed by this reference is shared by all
     * objects in any application.
     */
    CThreadServer *thread_server;
    /**
     * \brief identifier of the feedback thread
     *
     * This string has the identifier of the thread used to read status information from
     * the segway platform. This string is initialized at contruction time by appending
     * the unique identifier of the segway object and the string "_read_thread". This
     * thread is only used internally to the class, so it is not possible to get its
     * identifier out.
     */
    std::string read_thread_id;
    /**
     * \brief identifer of the command thread
     *
     * This string has the identifier of the thread used to send motion commands to 
     * the segway platform. This string is initialized at contruction time by appending
     * the unique identifier of the segway object and the string "_command_thread". This
     * thread is only used internally to the class, so it is not possible to get its
     * identifier out.
     */
    std::string command_thread_id;
    /**
     * \brief segway heartbeat thread
     *
     * This string has the idetifier of the thread used to check the periodic reception
     * of the heartbeat signal send by the segway platform. This string is initialized 
     * at contruction time by appending the unique identifier of the segway object and 
     * the string "_heartbeat_thread". This thread is only used internally to the class, 
     * so it is not possible to get its identifier out.
     */
    std::string heartbeat_thread_id;
    /**
     * \brief identifier of the reception event of the communication device
     * 
     * This string has the identifier of the event generated by the communcation device 
     * to signal de reception of new data. This event is not created by this class, but
     * retrieved from the communication device when it is attached to this class. This
     * event is used by the read thread to awake when new data is received. 
     */
    std::string comm_rx_event;
    /**
     * \brief identifier of the event to finish the feedback thread
     *
     * This string has the identifier of the event used to signal the feedback thread
     * to end its execution. This string is initialized at contruction time by appending
     * the unique identifier of the segway object and the string "_finish_read_thread".
     * This event is only used inside the class, so it is not possible to get its
     * identifier out.
     */
    std::string read_finish_event;
    /**
     * \brief identifier of the event to finish the command thread
     *
     * This string has the identifier of the event used to signal the command thread
     * to end its execution. This string is initialized at contruction time by appending
     * the unique identifier of the segway object and the string "_finish_command_thread".
     * This event is only used inside the class, so it is not possible to get its
     * identifier out.
     */
    std::string command_finish_event;
    /**
     * \brief identifier of the event to signal that the platform is powered off
     *
     * This string has the identifier of the event used to signal that the platform is
     * powered off. This string is initialized at contruction time by appending
     * the unique identifier of the segway object and the string "_power_off_event".
     * Use the get_power_off_event() function to get the event identifier from
     * outside the class.
     */
    std::string power_off_event;
    /**
     * \brief identifier of the event to signal that the cable is disconnected
     *
     * This string has the identifier of the event used to signal that the USB 
     * cable is disconnected. This string is initialized at contruction time by 
     * appending the unique identifier of the segway object and the string 
     * "_cable_disconnected_event". Use the get_cable_disconnected_event() 
     * function to get the event identifier from outside the class.
     */
    std::string cable_disconnected_event;
    /**
     * \brief identifier of the event to signal the availability of new status data
     *
     * This string has the identifier of the event used to signal the availability
     * of new status data. This string is initialized at contruction time by 
     * appending the unique identifier of the segway object and the string 
     * "_new_status_event". Use the get_new_status_event() function to 
     * get the event identifier from outside the class.
     */
    std::string new_status_event;
    /**
     * \brief identifeir of the event to signal the absence of the heartbeat signal
     *
     * This string has the identifier of the event used to signal that no heartbeat
     * has been received for a while. This string is initialized at contruction time 
     * by appending the unique identifier of the segway object and the string 
     * "_no_heartbeat_event". Use the get_no_heartbeat_event() function to 
     * get the event identifier from outside the class.
     */
    std::string no_heartbeat_event;
    /**
     * \brief identifeir of the internal heartbeat event
     *
     * This string has the identifier of the heartbeat event .This string is 
     * initialized at contruction time by appending the unique identifier of 
     * the segway object and the string "_heartbeat_event". This event is only 
     * used inside the class, so it is not possible to get its identifier out.
     */
    std::string heartbeat_event;
    /**
     * \brief class attributes initialization
     *
     * This method initalizes all the internal class variables at construction time.
     */
    void init_attributes(void);
    /**
     * \brief ftdi server initialization
     *
     * This method initializes the ftdi_server to handle all the FTDI devices  
     * and also adds the specific PID of the segway platforms in order to be able
     * to detect and list them.
     */
    void init_ftdi(void);
    /**
     * \brief function to initialize the class threads
     *
     * This method initializes all the internal threads of the class. This includes
     * the read and command threads and also the heartbeat thread. 
     * 
     * All the threads are created and assigned their corresponding execution function
     * but they are not started until the connection to the platform is done, either
     * at construction time or when the connect function is called.
     */
    void init_threads(void);
    /**
     * \brief function to initialize the class events
     * 
     * This function initializes all the internal and external events of the class.
     * 
     */
    void init_events(void);
  protected:
    // status variables
    /**
     * \brief left wheel velocity
     *
     * This value has the velocity of the left wheel in meters per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_left_wheel_velocity() function or the
     * overloaded operator <<.
     */
    float left_wheel_velocity;
    /**
     * \brief right wheel velovity
     *
     * This value has the velocity of the right wheel in meters per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_right_wheel_velocity() function or the
     * overloaded operator <<.
     */
    float right_wheel_velocity;
    /**
     * \brief pitch angle
     *
     * This value has the pitch angle of the platform in radians. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_pitch_angle() function or the
     * overloaded operator <<.
     */
    float pitch_angle;
    /**
     * \brief pitch rate
     *
     * This value has the pitch rate of the platform in radians per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_pitch_rate() function or the
     * overloaded operator <<.
     */
    float pitch_rate;
    /**
     * \brief roll angle
     *
     * This value has the roll angle of the platform in radians. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_roll_angle() function or the
     * overloaded operator <<.
     */
    float roll_angle;
    /**
     * \brief roll rate
     *
     * This value has the roll rate of the platform in radians per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_roll_rate() function or the
     * overloaded operator <<.
     */
    float roll_rate;
    /**
     * \brief yaw rate
     *
     * This value has the yaw rate of the platform in radians per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_yaw_rate() function or the
     * overloaded operator <<.
     */
    float yaw_rate;
    /**
     * \brief number of frames per second
     *
     * This value has the number of servo frames in fames per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_servo_frames() function or the
     * overloaded operator <<.
     */
    float servo_frames; 
    /**
     * \brief left wheel displacement
     *
     * This value has the displacement of the left wheel in meters. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_left_wheel_displacement() function 
     * or the overloaded operator <<.
     */
    float left_wheel_displ;
    /**
     * \brief right wheel displacement
     *
     * This value has the displacement of the right wheel in meters. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_right_wheel_displacement() function 
     * or the overloaded operator <<.
     */
    float right_wheel_displ;
    /**
     * \brief forward displacement
     *
     * This value has the displacement of the whole platform in meters. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_forward_displacement() function 
     * or the overloaded operator <<.
     */
    float forward_displ;
    /**
     * \brief yaw displacement
     *
     * This value has the rotation of the whole platform in radians. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_yaw_displacement() function 
     * or the overloaded operator <<.
     */
    float yaw_displ;
    /**
     * \brief left motor torque
     *
     * This value has the torque of the left motor in Newtorn per meter. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_left_motor_torque() function 
     * or the overloaded operator <<.
     */
    float left_torque;
    /**
     * \brief right motor torque
     *
     * This value has the torque of the right motor in Newtorn per meter. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_right_motor_torque() function 
     * or the overloaded operator <<.
     */
    float right_torque;
    /**
     * \brief operation mode
     *
     * This value has the current operation mode of the platform. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_operation_mode() function 
     * or the overloaded operator <<. The possible operation modes are:
     *
     * - tractor
     * - balance
     * - power down
     *
     * See the documentation of the op_mode data type for more information on the
     * possible operation modes.
     */
    op_mode mode;
    /**
     * \brief hardware operation mode
     *
     * This value has the current hardware operation mode of the platform. This 
     * value is periodically updated by the read thread and it is not possible to 
     * modify it otherwise. To get its value use the get_hardware_operation_mode() 
     * function or the overloaded operator <<. The possible operation modes are:
     *
     * - tractor
     * - balance
     * - power down
     *
     * See the documentation of the op_mode data type for more information on the
     * possible operation modes.
     *
     * This operation mode coincides with the platfrom settings (which of the two
     * blue buttons are on). This operation mode must coincide with the operation 
     * mode set by the user.
     */
    op_mode hardware_mode;
    /**
     * \brief gain schedule
     *
     * This value has the current gain schedule of the platform. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_gain_schedule() function 
     * or the overloaded operator <<. The possible gain schedules are:
     *
     * - light
     * - tall
     * - heavy.
     *
     * See the documentation of the gain data type for more information on the 
     * possible gain schedule.
     */
    gain gain_schedule;
    /**
     * \brief user battery
     *
     * This value has the voltage of the UI battery in Volts. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_ui_battery_voltage() function 
     * or the overloaded operator <<.
     */
    float ui_battery;
    /**
     * \brief power base battery
     *
     * This value has the voltage of the power base battery in Volts. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_powerbase_battery_voltage() function 
     * or the overloaded operator <<.
     */
    float powerbase_battery;
    // command variables
    /**
     * \brief translation velocity
     * 
     * This value has the desired translational velocity in meters per second. This 
     * values is updated each time the move() or stop() functions are called. Then
     * this value is sent periodically to the segway platform by the command thread.
     */
    short int vT; 
    /**
     * \brief rotation velocity
     *
     * This value has the desired rotational velocity in radians per second. This 
     * values is updated each time the move() or stop() functions are called. Then
     * this value is sent periodically to the segway platform by the command thread.
     */
    short int vR; 
    /**
     * \brief internal command queus
     *
     * This queue is used to store configuration commands to be send to the platform.
     * Initially, this queue is empty, and its length increases each time a set() or
     * reset() function is called. The command thread checks this queue every time it
     * is about to send a new command to include any configuration data if needed.
     */
    std::queue< std::vector<unsigned char> > command_queue;
    // methods
    /**
     * \brief function to compute checksum of USB packets 
     *
     * This function computes the checksum of a given USB packet either received from
     * the platform or to be send to it. When a new packet is received this function 
     * is used to check that the packet has no errors. Since the packet is sent with
     * a checksum, the return value of this function should be 0.
     *
     * When a packet is about to be sent, this function is used to compute the 
     * checksum value for the packet. The returned value then is used to complete
     * the data packet (18th bytes) before it is sent to the segway platform.
     *
     * This function is only used internally by the feedback and command threads,
     * but it can not be used elsewhere.
     *
     * \param packet a reference to the packet on which to compute the chcksum. If
     *               the packet has been received it should already has a checksum
     *               value at the 18th position, and otherwise this position should
     *               be set to 0 to correctly compute the checksum.
     *
     * \return the value of the checksum. If the packet has been received, this value 
     *         should be 0 for a valid packet and any other value otherwise. If the 
     *         packet is to be sent, this value is the checksum to be used to complete
     *         the packet.
     */		
    unsigned char compute_checksum(segway_packet *packet);
    /**
     * \brief function to read a whole packet from the segway platform
     *
     * This functions is used to read full packets from the segway platform. It first 
     * syncronizes the incomming data to detect the packet header, and then stores all
     * the data of the packet in the given packet strcuture. 
     *
     * This function reads all available data and returns immediatelly even if the 
     * whole packet has not been received. If several packets have been received, this
     * function returns after reading the first one, and the function has to be recalled
     * until no more data is available. This function is only used internally by the 
     * feedback thread.
     *
     * \param packet a reference to a packet structure in which to store the received 
     *               packet. The necessary memory for this structure has to be allocated
     *               before calling this function. This contents of this structure are
     *               only valid when the return value of this function is true, otherwise
     *               its contents should be ignored, but not modified since it has 
     *               partial packet information.
     * \param packet_len an integer with the size of the read packet. If the packet is 
     *                   completely received in a single call, this value is always 18 
     *                   (the dafault length). In case the packet is not completelly 
     *                   received in a single call, this parameter is used to hold the 
     *                   actual number of bytes read, and the same variable must be used 
     *                   in the next call to this function to guarantee a correct reception 
     *                   of all data packets.
     *
     * \return a flag that indicated if a whole packet has been read (true) or not (false).
     *         only when the return value is true, the contents of the packet parameter
     *         can be used.
     */
    bool read_packet(segway_packet *packet,int *packet_len);
    /**
     * \brief function to parse whole packets read from the segway platform
     *
     * This function is used to extract the data contained into the data packets received.
     * The checksum is first checked and if there is any error, an exception is thrown.
     * Otherwise, the data from the packet is stored into the corresponding internal 
     * attributes of the class.
     *
     * This function blocks the status mutex to avoid errors while updating the internal
     * attributes. This way all functions to get the current status will get blocked until
     * the update is complete.
     *
     * \param packet a reference to a packet structure which stores the data of a new 
     *               received packet. The packet passed to this function should be the
     *               one returned by the read_packet() function. 
     *
     */
    void parse_packet(segway_packet *packet);
    /**
     * \brief Thread function that reads data from segway
     *
     * This functions waits for any data to be received from the segway platform. When a whole
     * packet is read from the robot, it is parsed and then the internal information of the class
     * updated.
     *
     * This thread is created at construction time but it is not started until the communication
     * device is attached to avoid errors. This thread is always active until the finish event is
     * activated, which happens when the close() function is called.
     * 
     * \return in this case, the function always return NULL, so no data is returned to the 
     *         main thread.
     */
    void read_thread(void);
    /**
     * \brief start read thread
     *
     * This function starts the read thread.
     *
     * \param param a pointer to the object itself. since this function is statis, there exist no
     *              this pointer, so this parameter is used to access the internal attributes of
     *              the class.
     */
    static void *start_read_thread(void *param); 
    /**
     * \brief Thread function that sends commands to the segway
     *
     * This function is used to send motion commands to the segway platform periodically. This 
     * functions gets the internal motion commands (translational and rotational velocities) from
     * the class and sends it to the robot every 20ms. At this moment it is not possible to
     * change the update drate of the motion commandsi.
     * 
     * This thread also checks the internal command queue to check if there is any configuration
     * data avaialble. If so, it reads the contents of the first commans and includes it into the
     * packet data to be send. If no new configuration data is available, this thread only sends 
     * motion commands.
     *
     * This thread is created at construction time but it is not started until the communication
     * device is attached to avoid errors. This thread is always active until the finish event is
     * activated, which happens when the close() function is called. If this thread ends unexpectly
     * the robot will stop after a few moments, since it requires constant commands from the host
     * computer to continued operation.
     *
     * To change the current motion command, use the move() function. The stop() function should
     * be used to immediatelly stop the motors. These functions will modify the internal attributes
     * of the class, and this thread will be the one responsible of sending the new commands to
     * the robot.
     * 
     * \return in this case, the function always return NULL, so no data is returned to the 
     *         main thread.
     *
     */
    void command_thread(void);
    /**
     * \brief start command thread
     *
     * This function starts the command thread.
     *
     * \param param a pointer to the object itself. since this function is statis, there exist no
     *              this pointer, so this parameter is used to access the internal attributes of
     *              the class.
     */
    static void *start_command_thread(void *param);
    /**
     * \brief heartbeat thread function
     * 
     * This threads waits for the heartbeat event generated by the reception of a specific 
     * data packet from the segway platform for a given ammount of time.
     *
     * While this data packet is received the thread does nothing. When the communications
     * are interrupted or the power is turned off, the allowed ammount of time elapses 
     * and the external no_heartbeat_event is set to notify the error to the user. The
     * operation of this thread is equivalent to a watchdog.
     *
     * If the problem is solved, the event is reset and the normal operation of the 
     * driver continues.
     */
    static void *heartbeat_thread(void *param);
  public:
    /**
     * \brief constructor
     *
     * This constructor creates and initializes all the threads and events nedded by the class.
     * However, the threads are not started until the connect() function is called. This 
     * constructor accepts a string with the serial number of the segway platform, which is used to 
     * create the unique identifiers of both events and threads.
     * 
     * If no serial number is provided, the class tries to automatically connect to the 
     * platfrom. In this case is more or less than one platforms are connected to the
     * computer, the constructor throws an exception.
     *
     * \param desc_serial a null terminated string with the description or the serial
     *                    number of the segway platform to associate to the driver.
     *                    This string can be obtanied through the CFTDIServer or else
     *                    hardcoded if known.
     */
    CSegwayRMP200(const std::string& desc_serial="");
    /**
     * \brief function to get the identifeir of the segway
     *
     * This function is used to get the unique identifier of the segway platform associated
     * to the driver.
     *
     * \return a null terminated string with the identifier of the segway.
     */
    std::string get_id(void);
    /**
     * \brief function to get the identifier of the cable disconnected event
     *
     * This function returns the identifier of the cable disconnected event. This
     * event can be used by an external application to monitor the state of the
     * segway platform.
     */
    std::string get_cable_disconnected_event(void);
    /**
     * \brief function to get the identifier of the power off event
     *
     * This function returns the identifier of the power off event. This
     * event can be used by an external application to monitor the state of the
     * segway platform.
     */
    std::string get_power_off_event(void);
    /**
     * \brief function to get the no heartbeat event
     *
     * This function returns the identifier of the cable no heartbeat event. This
     * event can be used by an external applciation to monitor the state of the
     * segway platform.
     */
    std::string get_no_heartbeat_event(void);
    /**
     * \brief function to get the new status data available event
     *
     * This function returns the identifier of the new status data available event. 
     * This event can be used by an external application to know when there exist
     * new data, and avoid polling the driver too fast or too slowly.
     */
    std::string get_new_status_event(void);
    // configuration functions
    /**
     * \brief function to set the velocity scale factor
     *
     * This function is used to set the desired scale factor for the velocity. This scale factor 
     * is used to change the maximum velocity allowed. The given motion commands are multiplied
     * by this scale factor, thus limiting the maximum possible speed. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it.  
     *
     * \param factor the desired scale factor for the velocity. The possible values for this 
     *               parameter must be limited between 0.0 and 1.0. Any other value is not 
     *               allowed and will throw an exception.
     */
    void set_velocity_scale_factor(float factor);
    /**
     * \brief function to set the acceleration scale factor
     *
     * This function is used to set the desired scale factor for the acceleration. This scale factor 
     * is used to change the maximum acceleration allowed. The given motion commands are multiplied
     * by this scale factor, thus limiting the maximum possible acceleration. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it.  
     *
     * \param factor the desired scale factor for the acceleration. The possible values for this 
     *               parameter must be limited between 0.0 and 1.0. Any other value is not 
     *               allowed and will throw an exception.
     */
    void set_acceleration_scale_factor(float factor);
    /**
     * \brief function to set the turnrate scale factor
     *
     * This function is used to set the desired scale factor for the turn rate. This scale factor 
     * is used to change the maximum turn rate allowed. The given motion commands are multiplied
     * by this scale factor, thus limiting the maximum possible turn speed. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. 
     *
     * \param factor the desired scale factor for the turn rate. The possible values for this 
     *               parameter must be limited between 0.0 and 1.0. Any other value is not 
     *               allowed and will throw an exception
     */
    void set_turnrate_scale_factor(float factor);
    /**
     * \brief function to set the gain schedule
     *
     * This function is used to change the current gain schedule of the segway platform. The gain
     * schedule is used to improve the balance capabilities of the platform. Depending on the 
     * weight of the payload and on its distribution, this parameter must be changed. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it.  
     *
     * \param value the desired value for the gain schedule. This parameter should be one of the
     *              following values:
     *
     *              - light
     *              - tall
     *              - heavy
     *
     *              See the documentation on the gain data type for more information on the 
     *              different meaning of the gain schedules.
     */
    void set_gain_schedule(gain value);
    /**
     * \brief function to set the current limit scale factor
     *
     * This function is used to set the desired scale factor for the current limit. This scale 
     * factor is used to change the maximum motor current allowed. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it.  
     *
     * \param factor the desired scale factor for the current limit. The possible values for this 
     *               parameter must be limited between 0.0 and 1.0. Any other value is not 
     *               allowed and will throw an exception
     */
    void set_currentlimit_scale_factor(float factor);
    /**
     * \brief function to lock the balance function
     *
     * This function is used to block the balance mode. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     */
    void lock_balance(void);
    /**
     * \brief function to unlock the balance function
     *
     * This function is used to unblock the balance mode. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     */
    void unlock_balance(void);
    /**
     * \brief function to set the operation mode
     *
     * This function is used to change the current operation mode of the segway platform. The 
     * operation mode is used to change the working mode of the segway platform. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it.  
     *
     * \param mode the desired value for the operation mode. This parameter should be one of the
     *             following values:
     *
     *             - tractor
     *             - balance
     *             - power down
     *
     *             See the documentation on the op_mode data type for more information on the 
     *             different meaning of the operation modes.
     */
    void set_operation_mode(op_mode mode);
    /**
     * \brief function to reset the right wheel integrator
     *
     * This function is used to reeet the right wheel integrator of the segway platform. This 
     * integrator holds the total distance traveled by the right wheel. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it.  
     *     
     */
    void reset_right_wheel_integrator(void);
    /**
     * \brief function to reset the left wheel integrator
     *
     * This function is used to reset the left wheel integrator of the segway platform. This 
     * integrator holds the total distance traveled by the left wheel. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it.  
     */
    void reset_left_wheel_integrator(void);
    /**
     * \brief function to reset the yaw integrator
     *
     * This function is used to reset the yaw integrator of the segway platform. This 
     * integrator holds the total rotation performed by the robot. 
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it.  
     */
    void reset_yaw_integrator(void);
    /**
     * \brief function to reset the forward integrator
     *
     * This function is used to reset the forward integrator of the segway platform. This 
     * integrator holds the total forward displacement of the robot.
     *
     * This configuration information is processed but it is not immediatelly send to the platform.
     * Instead, it is stored internally and send by the command thread the next time a new motion
     * command has to be send. Therefore, there may exist a small delay between this call and the
     * actual change in the configuration. 
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it.  
     * 
     */
    void reset_forward_integrator(void);
    /**
     * \brief function to connect to the hardware platform
     *
     * This function is used to connect the driver with the hardware platform. The
     * communication device is only created and initialized when this function is 
     * called, so it is not possible to send or receive data to or from the platform
     * until then. When this function is called, both the command and feedback threads 
     * are started. Otherwise, the communication is not possible.
     *
     * At contruction time this function is automatically called so the segway is 
     * ready right after it is created. Howerver, it is possible to call this function
     * again in case the object msu be closed due to an error or because the target
     * platform changed.
     *
     * If a serial number is provided to this function, it tries to connect to the 
     * platform which has the same serial number. If no serial number is provided, it
     * tries to automatically connect to any platform it may find. If more or less than 
     * one segway platforms are connected to the computer, this function will throw
     * an exception.
     *
     * If the cable is disconnected, it throws a CSegwayCableDisconnectedException class and 
     * also activates the cable_diconnected_event event. If the power is turner off, it 
     * throws a CSegwayPowerOffException class and also activates the power_off_event event. 
     * Any other exception id rethrown without being processed.
     * This function throws a CSegwayRMP200Exception exception to report any error.
     *
     * \param desc_serial a null terminated string with the description or the serial
     *                    number of the segway platform to associate to the driver.
     *                    This string can be obtanied through the CFTDIServer or else
     *                    hardcoded if known.
     *
     */
    void connect(const std::string& desc_serial="");
    // status functions

    /**
     * \brief function to return the whole platform status
     *
     * This function returns the raw data status of all internal measures from
     * the platform.
     * 
     * \return struct with all data from platform sensors
     */
    TSegwayRMP200Status get_status(void);

    /**
     * \brief function to return the pitch angle
     *
     * This function returns the current pitch angle in radians. This function 
     * only returns the value of the internal attribute, but it does not access 
     * the hardware platform. This value is periodically updated by the feedback 
     * thread.
     *
     * \return the current pitch angle in radians.
     */
    float get_pitch_angle(void);
    /**
     * \brief function to return the pitch rate
     *
     * This function returns the current pitch rate in radians per second. This 
     * function only returns the value of the internal attribute, but it does not 
     * access the hardware platform. This value is periodically updated by the 
     * feedback thread.
     *
     * \return the current pitch rate in radians per second.
     */
    float get_pitch_rate(void);
    /**
     * \brief function to return the roll angle
     *
     * This function returns the current roll angle in radians. This function 
     * only returns the value of the internal attribute, but it does not access 
     * the hardware platform. This value is periodically updated by the feedback 
     * thread.
     *
     * \return the current roll angle in radians.
     */
    float get_roll_angle(void);
    /**
     * \brief function to return the roll rate
     *
     * This function returns the current roll rate in radians per second. This 
     * function only returns the value of the internal attribute, but it does not 
     * access the hardware platform. This value is periodically updated by the 
     * feedback thread.
     *
     * \return the current roll rate in radians per second.
     */
    float get_roll_rate(void);
    /**
     * \brief function to return the left wheel velocity
     *
     * This function returns the current left wheel velocity in meters per second. 
     * This function only returns the value of the internal attribute, but it 
     * does not access the hardware platform. This value is periodically updated 
     * by the feedback thread.
     *
     * \return the current left wheel in meters per second.
     */
    float get_left_wheel_velocity(void);
    /**
     * \brief function to return the right wheel velocity
     *
     * This function returns the current right wheel velocity in meters per second. 
     * This function only returns the value of the internal attribute, but it 
     * does not access the hardware platform. This value is periodically updated 
     * by the feedback thread.
     *
     * \return the current right wheel in meters per second.
     */
    float get_right_wheel_velocity(void);
    /**
     * \brief function to return the yaw rate
     *
     * This function returns the current yaw rate in radians per second. 
     * This function only returns the value of the internal attribute, but it 
     * does not access the hardware platform. This value is periodically updated 
     * by the feedback thread.
     *
     * \return the current yaw rate in radians per second.
     *
     */
    float get_yaw_rate(void);
    /**
     * \brief function to return the number of servo frames per second
     *
     * This function returns the current number of servo frames per second. This 
     * function only returns the value of the internal attribute, but it does
     * not access the hardware platform. This value is periodically updated by
     * the feedback thread.
     *
     * \return the current number of servo frames per second.
     */
    float get_servo_frames(void);
    /**
     * \brief function to return the left wheel displacement
     *
     * This function returns the current left wheel displacement in meters. This 
     * function only returns the value of the internal attribute, but it does
     * not access the hardware platform. This value is periodically updated by
     * the feedback thread.
     *
     * \return the current left wheel displacement in meters.
     */
    float get_left_wheel_displacement(void);
    /**
     * \brief function to return the right wheel displacement
     *
     * This function returns the current right wheel displacement in meters. This 
     * function only returns the value of the internal attribute, but it does
     * not access the hardware platform. This value is periodically updated by
     * the feedback thread.
     *
     * \return the current right wheel displacement in meters.
     */
    float get_right_wheel_displacement(void);
    /**
     * \brief function to return the total forward displacement
     *
     * This function returns the current forward displacement in meters. This 
     * function only returns the value of the internal attribute, but it does
     * not access the hardware platform. This value is periodically updated by
     * the feedback thread.
     *
     * \return the current forward displacement in meters.
     *
     */
    float get_forward_displacement(void);
    /**
     * \brief function to return the total yaw displacement
     *
     * This function returns the current yaw displacement in radians. This 
     * function only returns the value of the internal attribute, but it does 
     * not access the hardware platform. This value is periodically updated by 
     * the feedback thread.
     *
     * \return the current yaw displacement in radians per second.
     *
     */
    float get_yaw_displacement(void);
    /**
     * \brief function to return the current left motot torque
     *
     * This function returns the current left motor torque in Nm. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the current left motor torque in Nm.
     */
    float get_left_motor_torque(void);
    /**
     * \brief function to return the current right motor torque
     *
     * This function returns the current right motor torque in Nm. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the current right motor torque in Nm.
     *
     */
    float get_right_motor_torque(void);
    /**
     * \brief function to return the current operation mode
     *
     * This function returns the current operation mode of the segway platform. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the current operation mode being used. The possible values are:
     *
     * - tractor
     * - balance
     * - power down
     *
     */
    op_mode get_operation_mode(void);
    /**
     * \brief function to return the current hardware operation mode
     *
     * This function returns the current hardware operation mode of the segway 
     * platform. This function only returns the value of the internal attribute, 
     * but it does not access the hardware platform. This value is periodically 
     * updated by the feedback thread.
     * 
     * The value of this attribute is fixed by the state of the two blue buttons
     * on the platform. 
     *
     * \return the current operation mode being used. The possible values are:
     *
     * - tractor
     * - balance
     * - power down
     *
     */
    op_mode get_hardware_operation_mode(void);
    /**
     * \brief function to get the current gain schedule
     *
     * This function returns the current gain schedule of the segway platform. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the current gain schedule being used. The possible values are:
     *
     * - light
     * - tall
     * - heavy
     *
     */
    gain get_gain_schedule(void);
    /**
     * \brief function to return the value of the user battery voltage
     *
     * This function returns the current voltage of the user battery. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the value in volts of the user battery.
     */
    float get_ui_battery_voltage(void);
    /**
     * \brief function to get the current value of the powerbase battery voltage
     *
     * This function returns the current voltage of the powerbase battery. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the value in volts of the powerbase battery.
     *
     */
    float get_powerbase_battery_voltage(void);
    /**
     * \brief function to reset the segway platform
     *
     * This function is used to reset the segway platform to its default state. When
     * this function is called al configuration paramerets previoulsy set return to
     * its default value. Contrary to what happens with the motion commands, this 
     * function immediatelly sends a data packet to the segway platform.
     *
     * This function throws a CSegwayRMP200Exception exception to report any error.
     *
     */
    void reset(void);
    /**
     * \brief function to set new translational and rotational velocities
     *
     * This function is used to set a new motion command on the segway. This function 
     * sets the internal translational and rotational velocities to the desired values,
     * and it is the command thread which actually sends the new comamnd to the robot.
     *
     * This function can be called at any time to set up a new motion command to the 
     * robot. The command thread sends the current motion command to the robot every 
     * 20ms, so if this function is called more often, some of the commands will not
     * be executed. This function throws a CSegwayRMP200Exception to report errors.
     *
     * If the desired operation mode set by the user with the set_operation_mode()
     * function does not coincide with the hardware operation mode defined by the
     * state of the two blue buttons on the platform, this function throws an exception.
     * 
     * \param vT desired translational velocity in meters per second. This parameter
     *           is limited to 3.3 meters per second in both directions (both signs).
     *           However, the given value is affected by the velocity scale factor set
     *           by the set_velocity_scale_factor() function, so the actual maximum 
     *           speed can be lower.
     *
     * \param vR desired rotational velocity in radians per second. This parameter
     *           is limited to 0.01 radians per second in both directions (both
     *           signs). However, the given value is affectd by the turnrate scale 
     *           factor set by the set_turnrate_scale_factor() function, so the actual
     *           maximum turnrate can be lower.
     */
    void move(float vT,float vR);
    /**
     * \brief function to stop all motion
     *
     * This function is used to stop all motion on the segway. This function sets the
     * internal translational and rotational velocities to 0.0, and it is the command 
     * thread which actually sends the new comamnd to the robot.
     *
     * This function can be called at any time to stop the current motion of the robot.
     * This function throws a CSegwayRMP200Exception to report errors.
     *
     */
    void stop(void);
    /**
     * \brief function to close the segway driver
     *
     * This function stops all internal threads by activating the the associated finish 
     * events, but it does not destroy them, since it is possible to reconnect the object
     * to a new platform, in which case the thread will be restarted.
     *
     * This function also closes the communication device and frees all the associated
     * resources.
     *
     */
    void close(void);
    // operators
    /**
     * \brief display operator overloading
     *
     * This operator is used to show the current state of an object of this class
     * onto the standard ouput, file or any output stream. The information shown
     * is already formated as shown below:
     *
     * \verbatim
     * Pitch angle: <pitch_angle> radians
     * Pitch rate: <pitch_rate> radians/s
     * Roll angle: <roll_angle> radians
     * Roll rate: <roll_rate> radians/s
     * Left wheel velocity: <left_wheel_velocity> m/s
     * Right wheel velocity: <right_wheel_velocity> m/s
     * Yaw rate: <yaw_rate> radians/s
     * Servo frames: <servo_frames> frames/s
     * Left wheel displacement: <left_wheel_displ> m
     * Right wheel displacement: <right_wheel_displ> m
     * Forward displacement: <forward_displ> m
     * Yaw displacement: <yaw_displ> rev
     * Left motor torque: <left_torque> Nm
     * Right motor torque: <right_torque> Nm
     * Operation mode: <op_mode>
     * Hardware operation mode : <hardware_op_mode>
     * Gain schedule: <gain_schedule>
     * UI battery voltage: <ui_battery> V
     * Powerbase battery voltage: <powerbase_battery> V
     * \endverbatim
     */
    friend std::ostream& operator<< (std::ostream& out, CSegwayRMP200& segway);
    /**
     * \brief destructor
     *
     * This destructor is called when the object is about to be destroyed. It calls
     * the close() function to safely stop all the internal threads, and then deletes
     * all the internal threads and events.
     *
     */
    ~CSegwayRMP200(); 
};

#endif

