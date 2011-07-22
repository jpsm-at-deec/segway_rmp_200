#include "segway_rmp200.h"
#include "ftdiexceptions.h"
#include "eventserver.h"
#include "ftdiserver.h"
#include "ftdimodule.h"
#include <iostream>

/** \example test_segwayRMP200.cpp
 *
 * This example shows the basic operation of a Segway RMP200 platform.
 *
 * Before running this example it is important to follow the next steps:
 *
 * 	- check that E-Stop lanyard is plugged. (the red cordon). Motors won't
 * 	  start unless it is plugged
 *	- connect segway to USB slot of Pc host
 *	- start Segway UI processor (green button)
 *	- start Motors (yellow button)
 *
 * This example first creates an FTDI server to detect any possible segway
 * platform connected to the computer. It uses the add_custom_PID() function
 * to add the PID and VID combination used by Segway. It then opens the 
 * first device found, if any, using the serial number. At this point it is 
 * important that no other FTDI device is connected to the computer because 
 * the program does not check which kind of device it is.
 *
 * Once the FTDI device is opened and configured, a new CSegwayRMP200 object
 * is created an associated to the FTDI device. Then the robotic platform is
 * configured and commanded to move forward.
 *
 * While moving, every 1 second, the whole information of the platform is 
 * displayed on screen. Since the odometry of the robot uses inertial sensors,
 * if the platform is not actually moving, the displayed information will 
 * have no real meaning.
 *
 * It is important to note that it is not possible to execute any function 
 * of the public API of the CSegwayRMP200 class until a valid FTDI object 
 * has been associated to the segway object.
 *
 * If the motor are not enabled (yellow button turned off), the segway only
 * sends heartbeat commands, but it is not possible to send motion commands
 * or receive feedback data. In this case, the program will not fail, but
 * nothing will happen.
 */

std::string segway_name="segway"; 

int main(int argc, char *argv[])
{
  CSegwayRMP200 *segway = NULL;
  std::list<std::string> events;
  CEventServer *event_server=CEventServer::instance();
  std::string serial_number,cable_disconnected,power_off;
  bool connected=false;
  int i=0,event_id=0;

  try
  {
    while(!connected)
    {
      try
      {
        segway = new CSegwayRMP200();
        connected = true;
      }
      catch(CException &e)
      {
        if(segway != NULL)
          delete segway;

        segway = NULL;

        std::cout << e.what() << std::endl;
        std::cout << "The segway platfrom is still not ready (is it power on?)" << std::endl << std::endl;
        sleep(1);
      }
    }

    segway->unlock_balance();
    segway->set_operation_mode(tractor);
    segway->set_gain_schedule(light);
    segway->reset_right_wheel_integrator();
    segway->reset_left_wheel_integrator();
    segway->reset_yaw_integrator();
    segway->reset_forward_integrator();

    // keep in mind that the order in of the event son the events list fix their
    // priority. The first event on the list will be checked first, and if it is 
    // active, the other will not be checked.
    events.push_back(segway->get_cable_disconnected_event());    
    events.push_back(segway->get_power_off_event());    
    events.push_back(segway->get_no_heartbeat_event());    
    events.push_back(segway->get_new_status_event());   

    segway->move(0.1,0.0);
    for(i=0;i<1000000;i++)
    {
      event_id=event_server->wait_first(events);
      if(event_id==3)
      {
        if((i%100)==0)
          std::cout << (*segway) << std::endl;
      }
      else if(event_id==2)
      {
        std::cout << "No heart beat detected" << std::endl;
        segway->stop();
        sleep(1);
      }
      else if(event_id==1)
      {
        std::cout << "The segway platform power is off" << std::endl;
        segway->stop();
        sleep(1);
      }
      else
      {
        std::cout << "The USB cable has been disconnected" << std::endl;
        segway->stop();
        segway->close();
        connected=false;
        while(!connected)
        {
          try
          {
            segway->connect();
            connected=true;
            segway->move(0.1,0.0);
          }
          catch(CException &e)
          {
            std::cout << "The segway platfrom is still not ready" << std::endl;
            sleep(1);
          }
        }
      }
    }
    segway->stop();
    segway->close();
    delete segway;
  }
  catch(CException &e)
  {
     std::cout << e.what() << std::endl;
  }
}
