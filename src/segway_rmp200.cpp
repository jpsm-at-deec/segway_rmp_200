#include "segway_rmp200.h"
#include "segway_rmp200_exceptions.h"
#include "eventexceptions.h"
#include "ftdiexceptions.h"
#include <iostream>
#include <string>

const std::string CSegwayRMP200::description = "Robotic Mobile Platform";
const short int CSegwayRMP200::pid = 0xE729;

CSegwayRMP200::CSegwayRMP200(const std::string& desc_serial)
{
  std::string serial_number=desc_serial;
  std::vector<int> ftdi_devs;

  this->event_server  = CEventServer::instance(); 
  this->thread_server = CThreadServer::instance();

  this->init_attributes();
  this->init_ftdi();

  if(desc_serial.compare("") == 0)
  {
    ftdi_devs = this->ftdi_server->get_ids_by_description(CSegwayRMP200::description);
    
    //check how many segway devices are connected
    //launch exception if there are no segways or there is more than one
    if(ftdi_devs.size() != 1)
    {
      if(ftdi_devs.size() == 0)
        throw CSegwayRMP200Exception(_HERE_, "No segways available", this->id);
      else
      {
        throw CSegwayRMP200Exception(_HERE_, "More than one segway available", this->id);
      }
    }
    serial_number= this->ftdi_server->get_serial_number(ftdi_devs.at(0));
  }
  this->id = "segway_" + serial_number;
  this->init_threads();
  this->init_events();

  this->connect(desc_serial);
}

void CSegwayRMP200::init_attributes(void)
{
  // initialize internal variables
  this->left_wheel_velocity=0.0;
  this->right_wheel_velocity=0.0;
  this->pitch_angle=0.0;
  this->pitch_rate=0.0;
  this->roll_angle=0.0;
  this->roll_rate=0.0;
  this->yaw_rate=0.0;
  this->servo_frames=0.0;
  this->left_wheel_displ=0.0;
  this->right_wheel_displ=0.0;
  this->forward_displ=0.0;
  this->yaw_displ=0.0;
  this->left_torque=0.0;
  this->right_torque=0.0;
  this->mode=(op_mode)-1;
  this->hardware_mode=(op_mode)-1;
  this->gain_schedule=(gain)-1;
  this->ui_battery=0.0;
  this->powerbase_battery=0.0;
  // command variables
  this->vT=0; 
  this->vR=0;
}

void CSegwayRMP200::init_ftdi(void)
{
    this->comm_dev=NULL;
    this->ftdi_server = CFTDIServer::instance();
    this->ftdi_server->add_custom_PID(this->pid);
}

void CSegwayRMP200::init_threads(void)
{

  // create the feedback and command threads
  this->read_thread_id = this->id;
  this->read_thread_id += "_read_thread";
  this->thread_server->create_thread(this->read_thread_id);
  this->thread_server->attach_thread(this->read_thread_id,this->start_read_thread,this);
  this->command_thread_id = this->id;
  this->command_thread_id += "_command_thread";
  this->thread_server->create_thread(this->command_thread_id);
  this->thread_server->attach_thread(this->command_thread_id,this->start_command_thread,this);
  //create the heartbeat thread
  this->heartbeat_thread_id = this->id;
  this->heartbeat_thread_id += "_heartbeat_thread";
  this->thread_server->create_thread(this->heartbeat_thread_id);
  this->thread_server->attach_thread(this->heartbeat_thread_id,this->heartbeat_thread,this);
}

void CSegwayRMP200::init_events(void)
{
  // create the finish events
  this->read_finish_event = this->id;
  this->read_finish_event += "_finish_read_thread";
  this->event_server->create_event(this->read_finish_event);
  this->command_finish_event = this->id;
  this->command_finish_event += "_finish_command_thread";
  this->event_server->create_event(this->command_finish_event);
  // create the error events
  this->cable_disconnected_event = this->id;
  this->cable_disconnected_event += "_cable_disconnected";
  this->event_server->create_event(this->cable_disconnected_event);
  this->power_off_event = this->id;
  this->power_off_event += "_power_off_event";
  this->event_server->create_event(this->power_off_event);
  this->no_heartbeat_event = this->id;
  this->no_heartbeat_event += "_no_heartbeat";
  this->event_server->create_event(this->no_heartbeat_event);
  // create the heartbeat event
  this->heartbeat_event = this->id;
  this->heartbeat_event += "_heartbeat";
  this->event_server->create_event(this->heartbeat_event);
  // create the new status event
  this->new_status_event = this->id;
  this->new_status_event += "_new_status";
  this->event_server->create_event(this->new_status_event);

}

std::string CSegwayRMP200::get_id(void)
{
  return this->id;
}

std::string CSegwayRMP200::get_cable_disconnected_event(void)
{
  return this->cable_disconnected_event;
}

std::string CSegwayRMP200::get_power_off_event(void)
{
  return this->power_off_event;
}

std::string CSegwayRMP200::get_no_heartbeat_event(void)
{
  return this->no_heartbeat_event;
}

std::string CSegwayRMP200::get_new_status_event(void)
{
  return this->new_status_event;
}

unsigned char CSegwayRMP200::compute_checksum(segway_packet *packet)
{
  int i;

  unsigned short int checksum=0;
  unsigned short int checksum_high=0;

  for(i=0;i<17;i++)
    checksum += (short)packet->data[i];
  checksum_high = (unsigned short)(checksum >> 8);
  checksum &= 0xFF;
  checksum +=checksum_high;
  checksum_high = (unsigned short)(checksum >> 8);
  checksum &= 0xFF;
  checksum +=checksum_high;
  checksum = (~checksum + 1) & 0xFF;
 
  return (unsigned char)checksum;
}

bool CSegwayRMP200::read_packet(segway_packet *packet,int *packet_len)
{
  int read_data=0,len=0,pos=0;
  unsigned char *data=NULL;

  if(packet==NULL)
  {
    /* handle exceptions */
    throw CSegwayRMP200Exception(_HERE_,"Invalid packet structure",this->id);
  } 
  else
  {
    len=this->comm_dev->get_num_data();
    data=new unsigned char[len];
    try{
      read_data=this->comm_dev->read(data,len);
    }catch(CException &e){
      /* handle exception */
      throw;
    }
    if(read_data!=len)
    {
      /* handle exceptions */
      throw CSegwayRMP200Exception(_HERE_,"Unexpected error while reading the USB device",this->id);
    }
    while(pos<len)
    {
      if((*packet_len)==0)
      {
        if(data[pos]==0xF0)
        {
          packet->data[(*packet_len)]=data[pos];
          (*packet_len)++;
        }
      }
      else
      { 
        if((*packet_len)==1)
        {
          if(data[pos]==0x55)
          {
            packet->data[(*packet_len)]=data[pos];
            (*packet_len)++;
          }
          else
            (*packet_len)=0;
        }
        else
        {
          if((*packet_len)==2)
          {
            if(data[pos]==0xAA || data[pos]==0xBB)
            {
              packet->data[(*packet_len)]=data[pos];
              (*packet_len)++;
            }
            else
              (*packet_len)=0;
          }
          else
          {
            packet->data[(*packet_len)]=data[pos];
            (*packet_len)++;
            if((*packet_len)==18)
            {
              (*packet_len)=0;
              if(data!=NULL)
                delete[] data;
              return true;
            }
          }
        }
      }
      pos++;
    }
  }

  if(data!=NULL)
    delete[] data;
  return false;
}

void CSegwayRMP200::parse_packet(segway_packet *packet)
{
  short int command;
  int i=0;

  if(this->compute_checksum(packet)!=packet->data[17])
  {
    /* handle exceptions */
    throw CSegwayRMP200Exception(_HERE_,"Invalid packet checksum, a transmission error ocurred",this->id);
  }
  if(packet->data[2]==0xaa)
  {
    this->access_status.enter();
    command=(packet->data[4]*256+packet->data[5])/32;
    switch(command)
    {
      case 0x0400: break;
      case 0x0401: this->pitch_angle=((float)((short int)(((int)packet->data[9]<<8)+(int)packet->data[10])))/7.8;
                   this->pitch_rate=((float)((short int)(((int)packet->data[11]<<8)+(int)packet->data[12])))/7.8;
                   this->roll_angle=((float)((short int)(((int)packet->data[13]<<8)+(int)packet->data[14])))/7.8;
                   this->roll_rate=((float)((short int)(((int)packet->data[15]<<8)+(int)packet->data[16])))/7.8;
                   break;
      case 0x0402: this->left_wheel_velocity=((float)((short int)(((int)packet->data[9]<<8)+(int)packet->data[10])))/332.0;
                   this->right_wheel_velocity=((float)((short int)(((int)packet->data[11]<<8)+(int)packet->data[12])))/332.0;
                   this->yaw_rate=((float)((short int)(((int)packet->data[13]<<8)+(int)packet->data[14])))/7.8;
                   this->servo_frames=((float)((unsigned short int)(((int)packet->data[15]<<8)+(int)packet->data[16])))*0.01;
                   break; 
      case 0x0403: this->left_wheel_displ=((float)((int)(((int)packet->data[11]<<24)+((int)packet->data[12]<<16)+((int)packet->data[9]<<8)+(int)packet->data[10])))/33215.0; 
                   this->right_wheel_displ=((float)((int)(((int)packet->data[15]<<24)+((int)packet->data[16]<<16)+((int)packet->data[13]<<8)+(int)packet->data[14])))/33215.0;
                   break;
      case 0x0404: this->forward_displ=((float)((int)(((int)packet->data[11]<<24)+((int)packet->data[12]<<16)+((int)packet->data[9]<<8)+(int)packet->data[10])))/33215.0;;
                   this->yaw_displ=((float)((int)(((int)packet->data[15]<<24)+((int)packet->data[16]<<16)+((int)packet->data[13]<<8)+(int)packet->data[14])))/112644.0;
                   break;
      case 0x0405: this->left_torque=((float)((short int)(((int)packet->data[9]<<8)+(int)packet->data[10])))/1094.0;
                   this->right_torque=((float)((short int)(((int)packet->data[11]<<8)+(int)packet->data[12])))/1094.0;
                   break;
      case 0x0406: this->mode=(op_mode)(packet->data[9]*256+packet->data[10]);
                   this->gain_schedule=(gain)(((int)packet->data[11]<<8)+(int)packet->data[12]);
                   this->ui_battery=1.4+(float)(((int)packet->data[13]<<8)+(int)packet->data[14])*0.0125;
                   this->powerbase_battery=(float)(((int)packet->data[15]<<8)+(int)packet->data[16])/4.0;
                   if(!this->event_server->event_is_set(this->new_status_event))
                     this->event_server->set_event(this->new_status_event);
                   break;
      case 0x0407: break;
      case 0x0680: this->hardware_mode=(op_mode)(packet->data[10]&0x03);
                   if(!this->event_server->event_is_set(this->heartbeat_event))
                     this->event_server->set_event(this->heartbeat_event);
                   break;
      case 0x0688: break;
      default: /* handle exceptions */
               break;
    }
    this->access_status.exit();
  }
}

void *CSegwayRMP200::start_read_thread(void *param)
{
  CSegwayRMP200 *segway = (CSegwayRMP200 *) param;

  segway->read_thread();
}

void CSegwayRMP200::read_thread(void)
{
  std::list<std::string> event_list;
  int packet_len=0,event_id;
  segway_packet packet;
  bool end=false;

  event_list.push_back(this->comm_rx_event);
  event_list.push_back(this->read_finish_event);
  while(!end)
  {
    event_id=this->event_server->wait_first(event_list);
    if(event_id==0)
    {
      try{
        if(this->read_packet(&packet,&packet_len))
          this->parse_packet(&packet);
      }catch(CException &e){
        std::cout << e.what() << std::endl;
        throw;
      }
    }
    else
    {
      end=true; 
    }
  }

  pthread_exit(NULL);
}

void *CSegwayRMP200::start_command_thread(void *param)
{
  CSegwayRMP200 *segway = (CSegwayRMP200 *) param;

  segway->command_thread();
}
 
void CSegwayRMP200::command_thread(void)
{
  segway_packet packet={0xF0,0x55,0x00,0x00,0x00,0x00,0x04,0x13,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  std::vector<unsigned char> command;
  int exception_count=0;
  bool end=false;
  short int vT,vR;

  while(!end)
  {
    usleep(100000);
    this->access_command.enter();
    vT=this->vT;
    vR=this->vR;
    this->access_command.exit();
    packet.data[9]=((unsigned char *)&vT)[1];
    packet.data[10]=((unsigned char *)&vT)[0];
    packet.data[11]=((unsigned char *)&vR)[1];
    packet.data[12]=((unsigned char *)&vR)[0];
    this->access_command.enter();
    if(this->command_queue.size()>0)
    {
      command=this->command_queue.front();
      this->command_queue.pop();
      packet.data[13]=command[0];
      packet.data[14]=command[1];
      packet.data[15]=command[2];
      packet.data[16]=command[3];
    }
    this->access_command.exit();
    packet.data[17]=this->compute_checksum(&packet);
    try{
      this->comm_dev->write(packet.data,18);
      if(this->event_server->event_is_set(this->cable_disconnected_event))
        this->event_server->reset_event(this->cable_disconnected_event);
      if(this->event_server->event_is_set(this->power_off_event))
        this->event_server->reset_event(this->power_off_event);
    }catch(CFTDIException &e){// cable disconnected
      exception_count=0;
      if(!this->event_server->event_is_set(this->cable_disconnected_event))
        this->event_server->set_event(this->cable_disconnected_event);
    }catch(CCommException &e){// power off and also cable disconnected
      exception_count++;
      if(exception_count==2)
      {
        exception_count=0;
        if(!this->event_server->event_is_set(this->power_off_event))
          this->event_server->set_event(this->power_off_event);
      }
    }catch(CException &e){// something else
      std::cout << e.what() << std::endl;
      throw;
    }
    if(this->event_server->event_is_set(this->command_finish_event))
    {
      this->event_server->reset_event(this->command_finish_event);
      end=true;
    }
  }

  pthread_exit(NULL);
}

void *CSegwayRMP200::heartbeat_thread(void *param)
{
  CSegwayRMP200 *segway = (CSegwayRMP200 *) param;
  std::list<std::string> events;

  events.push_back(segway->heartbeat_event);
  while(1)
  {
    try{
      segway->event_server->wait_all(events,200);
    }catch(CEventTimeoutException &e){
      if(!segway->event_server->event_is_set(segway->no_heartbeat_event))
        segway->event_server->set_event(segway->no_heartbeat_event);
    }
  }
  
  pthread_exit(NULL); 
}

// configuration functions
void CSegwayRMP200::set_velocity_scale_factor(float factor)
{
  std::vector<unsigned char> command;
  short int value;

  if(factor < 0.0 || factor > 1.0)
  {
    /* handle exceptions */
    throw CSegwayRMP200Exception(_HERE_,"Invalid scale factor for velocity",this->id);
  }
  else
  {
    command.resize(4);
    value=(short int)(factor*16);
    command[0]=0;
    command[1]=10;
    command[2]=((unsigned char *)&value)[1];
    command[3]=((unsigned char *)&value)[0];
    this->access_command.enter();
    this->command_queue.push(command);
    this->access_command.exit();
  }
}

void CSegwayRMP200::set_acceleration_scale_factor(float factor)
{
  std::vector<unsigned char> command;
  short int value;

  if(factor < 0.0 || factor > 1.0)
  {
    /* handle exceptions */
    throw CSegwayRMP200Exception(_HERE_,"Invalid scale factor for acceleration",this->id);
  }
  else
  {
    command.resize(4);
    value=(short int)(factor*16);
    command[0]=0;
    command[1]=11;
    command[2]=((unsigned char *)&value)[1];
    command[3]=((unsigned char *)&value)[0];
    this->access_command.enter();
    this->command_queue.push(command);
    this->access_command.exit();
  }
}

void CSegwayRMP200::set_turnrate_scale_factor(float factor)
{
  std::vector<unsigned char> command;
  short int value;

  if(factor < 0.0 || factor > 1.0)
  {
    /* handle exceptions */
    throw CSegwayRMP200Exception(_HERE_,"Invalid scale factor for turn rate",this->id);
  }
  else
  {
    command.resize(4);
    value=(short int)(factor*16);
    command[0]=0;
    command[1]=12;
    command[2]=((unsigned char *)&value)[1];
    command[3]=((unsigned char *)&value)[0];
    this->access_command.enter();
    this->command_queue.push(command);
    this->access_command.exit();
  }
}

void CSegwayRMP200::set_gain_schedule(gain value)
{
  std::vector<unsigned char> command;

  command.resize(4);
  command[0]=0;
  command[1]=13;
  command[2]=((unsigned char *)&value)[1];
  command[3]=((unsigned char *)&value)[0];
  this->access_command.enter();
  this->command_queue.push(command);
  this->access_command.exit();
}

void CSegwayRMP200::set_currentlimit_scale_factor(float factor)
{
  std::vector<unsigned char> command;
  short int value;

  if(factor < 0.0 || factor > 1.0)
  {
    /* handle exceptions */
    throw CSegwayRMP200Exception(_HERE_,"Invalid scale factor for current limit",this->id);
  }
  else
  {
    command.resize(4);
    value=(short int)(factor*256);
    command[0]=0;
    command[1]=14;
    command[2]=((unsigned char *)&value)[1];
    command[3]=((unsigned char *)&value)[0];
    this->access_command.enter();
    this->command_queue.push(command);
    this->access_command.exit();
  }
}

void CSegwayRMP200::lock_balance(void)
{
  std::vector<unsigned char> command;

  command.resize(4);
  command[0]=0;
  command[1]=15;
  command[2]=0;
  command[3]=1;
  this->access_command.enter();
  this->command_queue.push(command);
  this->access_command.exit();
}

void CSegwayRMP200::unlock_balance(void)
{
  std::vector<unsigned char> command;

  command.resize(4);
  command[0]=0;
  command[1]=15;
  command[2]=0;
  command[3]=0;
  this->access_command.enter();
  this->command_queue.push(command);
  this->access_command.exit();
}

void CSegwayRMP200::set_operation_mode(op_mode mode)
{
  std::vector<unsigned char> command;
std::cout << "set_operation_mode::hardware_mode=" << hardware_mode << " vs. mode=" << mode << std::endl;
  if(this->hardware_mode==mode)
  {
    command.resize(4);
    command[0]=0;
    command[1]=16;
    command[2]=((unsigned char *)&mode)[1];
    command[3]=((unsigned char *)&mode)[0];
    this->access_command.enter();
    this->command_queue.push(command);
    this->access_command.exit();
  }
  else
  {
    /* handle exceptions */
    throw CSegwayRMP200Exception(_HERE_,"The platform current operation mode does not coincide with the desiered mode",this->id);
  }
}

void CSegwayRMP200::reset_right_wheel_integrator(void)
{
  std::vector<unsigned char> command;

  command.resize(4);
  command[0]=0;
  command[1]=50;
  command[2]=0;
  command[3]=1;
  this->access_command.enter();
  this->command_queue.push(command);
  this->access_command.exit();
}

void CSegwayRMP200::reset_left_wheel_integrator(void)
{
  std::vector<unsigned char> command;

  command.resize(4);
  command[0]=0;
  command[1]=50;
  command[2]=0;
  command[3]=2;
  this->access_command.enter();
  this->command_queue.push(command);
  this->access_command.exit();
}

void CSegwayRMP200::reset_yaw_integrator(void)
{
  std::vector<unsigned char> command;

  command.resize(4);
  command[0]=0;
  command[1]=50;
  command[2]=0;
  command[3]=4;
  this->access_command.enter();
  this->command_queue.push(command);
  this->access_command.exit();
}

void CSegwayRMP200::reset_forward_integrator(void)
{
  std::vector<unsigned char> command;

  command.resize(4);
  command[0]=0;
  command[1]=50;
  command[2]=0;
  command[3]=8;
  this->access_command.enter();
  this->command_queue.push(command);
  this->access_command.exit();
} 

TSegwayRMP200Status CSegwayRMP200::get_status(void)
{
    TSegwayRMP200Status status;

    status.right_wheel_velocity = this-> right_wheel_velocity;
    status.left_wheel_velocity  = this-> left_wheel_velocity;
    status.pitch_angle          = this-> pitch_angle;
    status.pitch_rate           = this-> pitch_rate;
    status.roll_angle           = this-> roll_angle;
    status.roll_rate            = this-> roll_rate;
    status.yaw_rate             = this-> yaw_rate;
    status.left_wheel_displ     = this-> left_wheel_displ;
    status.right_wheel_displ    = this-> right_wheel_displ;
    status.forward_displ        = this-> forward_displ;
    status.yaw_displ            = this-> yaw_displ;
    status.servo_frames         = this-> servo_frames;
    status.left_torque          = this-> left_torque;
    status.right_torque         = this-> right_torque;
    status.ui_battery           = this-> ui_battery;
    status.powerbase_battery    = this-> powerbase_battery;
    op_mode operation_mode      = this-> mode;
    op_mode hardware_mode       = this-> hardware_mode;
    gain gain_schedule          = this-> gain_schedule;

    return status;
}

// status functions
float CSegwayRMP200::get_pitch_angle(void)
{
  return this->pitch_angle;
}

float CSegwayRMP200::get_pitch_rate(void)
{
  return this->pitch_rate;
}

float CSegwayRMP200::get_roll_angle(void)
{
  return this->roll_angle;
}

float CSegwayRMP200::get_roll_rate(void)
{
  return this->roll_rate;
}

float CSegwayRMP200::get_left_wheel_velocity(void)
{
  return this->left_wheel_velocity;
}

float CSegwayRMP200::get_right_wheel_velocity(void)
{
  return this->right_wheel_velocity;
}

float CSegwayRMP200::get_yaw_rate(void)
{
  return this->yaw_rate;
}

float CSegwayRMP200::get_servo_frames(void)
{
  return this->servo_frames;
}

float CSegwayRMP200::get_left_wheel_displacement(void)
{
  return this->left_wheel_displ;
}

float CSegwayRMP200::get_right_wheel_displacement(void)
{
  return this->right_wheel_displ;
}

float CSegwayRMP200::get_forward_displacement(void)
{
  return this->forward_displ;
}

float CSegwayRMP200::get_yaw_displacement(void)
{
  return this->yaw_displ;
}

float CSegwayRMP200::get_left_motor_torque(void)
{
  return this->left_torque;
}

float CSegwayRMP200::get_right_motor_torque(void)
{
  return this->right_torque;
}

op_mode CSegwayRMP200::get_operation_mode(void)
{
  return this->mode;
}

op_mode CSegwayRMP200::get_hardware_operation_mode(void)
{
  return this->hardware_mode;
}

gain CSegwayRMP200::get_gain_schedule(void)
{
  return this->gain_schedule;
}

float CSegwayRMP200::get_ui_battery_voltage(void)
{
  return this->ui_battery;
}

float CSegwayRMP200::get_powerbase_battery_voltage(void)
{
  return this->powerbase_battery;
}

void CSegwayRMP200::connect(const std::string& desc_serial)
{
  std::string serial_number = desc_serial;
  std::list<std::string> events;

  // rescan the bus to update the local information
  this->ftdi_server->scan_bus();

  //if no serial number is provided
  if(desc_serial.compare("") == 0)
  {
    std::vector<int> ftdi_devs;
    ftdi_devs = this->ftdi_server->get_ids_by_description(CSegwayRMP200::description);
    
    //check how many segway devices are connected
    //launch exception if there are no segways or there is more than one
    if(ftdi_devs.size() != 1)
    {
      if(ftdi_devs.size() == 0)
        throw CSegwayRMP200Exception(_HERE_, "No segways available", this->id);
      else
      {
        throw CSegwayRMP200Exception(_HERE_, "More than one segway available", this->id);
      }
    }
    serial_number = this->ftdi_server->get_serial_number(ftdi_devs.at(0));
  }

  TFTDIconfig ftdi_config;

  if(this->comm_dev!=NULL)
    this->close();

  this->comm_dev=this->ftdi_server->get_device(serial_number);

  ftdi_config.baud_rate     = 460800;
  ftdi_config.word_length   = -1;
  ftdi_config.stop_bits     = -1;
  ftdi_config.parity        = -1;
  ftdi_config.read_timeout  = 1000;
  ftdi_config.write_timeout = 1000;
  ftdi_config.latency_timer = 1;

  this->comm_dev->config(&ftdi_config);
  this->comm_rx_event=this->comm_dev->get_rx_event_id();
  this->thread_server->start_thread(this->read_thread_id);
  this->thread_server->start_thread(this->command_thread_id);
  

  // wait until the state has been updated for the first time
  events.push_back(this->new_status_event);
  events.push_back(this->heartbeat_event);
  try
  {
    this->event_server->wait_all(events, 2000);
    this->event_server->wait_all(events, 2000);
  }
  catch(CEventTimeoutException e)
  {
    this->close();
    throw CSegwayRMP200Exception(_HERE_,"Segway connection not ready",this->id);
  }

  this->thread_server->start_thread(this->heartbeat_thread_id);
}

void CSegwayRMP200::reset(void)
{
  segway_packet packet={0xF0,0x55,0x00,0x00,0x00,0x00,0x04,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

  if(this->comm_dev==NULL)
  {
    /* handle exceptions */
    throw CSegwayRMP200Exception(_HERE_,"Communciation device not set",this->id);
  }
  else
  {
    packet.data[17]=this->compute_checksum(&packet);
    this->comm_dev->write(packet.data,18);
  }

}
	
void CSegwayRMP200::move(float vT,float vR)
{
std::cout << "move::hardware_mode=" << hardware_mode << " vs. mode=" << mode << std::endl;
  if(this->mode!=this->hardware_mode)
  {
    /* handle exceptions */
    throw CSegwayRMP200Exception(_HERE_,"The desired operation mode does not coincide with the one set in the platform.",this->id);
  }
  else
  {
    this->access_command.enter();
    this->vT = (short int)(vT*3.6*147.0/1.609344);
    this->vR = (short int)(vR*1024.0);
    this->access_command.exit();
  }
}

void CSegwayRMP200::stop(void)
{
  this->access_command.enter();
  this->vT = 0;
  this->vR = 0;
  this->access_command.exit();
}

void CSegwayRMP200::close()
{
  //kill threads
  if(this->comm_dev!=NULL)
  {
    // finish the threads
    this->event_server->set_event(this->read_finish_event);
    this->event_server->set_event(this->command_finish_event);
    this->thread_server->end_thread(this->read_thread_id);
    this->thread_server->end_thread(this->command_thread_id);
    this->thread_server->kill_thread(this->heartbeat_thread_id);
    /* reset the events if necessary */
    this->event_server->reset_event(this->read_finish_event);
    this->event_server->reset_event(this->command_finish_event);
    if(this->event_server->event_is_set(this->no_heartbeat_event))
      this->event_server->reset_event(this->no_heartbeat_event);
    if(this->event_server->event_is_set(this->power_off_event))
      this->event_server->reset_event(this->power_off_event);
    if(this->event_server->event_is_set(this->cable_disconnected_event))
      this->event_server->reset_event(this->cable_disconnected_event);
    this->comm_dev->close();
    delete this->comm_dev;
    this->comm_dev=NULL;
  }
}

CSegwayRMP200::~CSegwayRMP200()
{
  this->close();
  /* destroy the events */
  this->event_server->delete_event(this->read_finish_event);
  this->read_finish_event="";
  this->event_server->delete_event(this->command_finish_event);
  this->command_finish_event="";
  this->event_server->delete_event(this->cable_disconnected_event);
  this->cable_disconnected_event="";
  this->event_server->delete_event(this->power_off_event);
  this->power_off_event="";
  this->event_server->delete_event(this->no_heartbeat_event);
  this->no_heartbeat_event="";
  this->event_server->delete_event(this->heartbeat_event);
  this->heartbeat_event="";
  this->event_server->delete_event(this->new_status_event);
  this->new_status_event="";
  // destroy the threads
  this->thread_server->delete_thread(this->read_thread_id);
  this->read_thread_id="";
  this->thread_server->delete_thread(this->command_thread_id);
  this->command_thread_id="";
  this->thread_server->delete_thread(this->heartbeat_thread_id);
  this->heartbeat_thread_id="";
}

std::ostream& operator<< (std::ostream& out, CSegwayRMP200& segway)
{
  segway.access_status.enter();
  out << "Pitch angle: " << segway.pitch_angle << " degrees" << std::endl;
  out << "Pitch rate " << segway.pitch_rate << " degrees/s" << std::endl;
  out << "Roll angle: " << segway.roll_angle << " degrees" << std::endl;
  out << "Roll rate: " << segway.roll_rate << " degrees/s" << std::endl;
  out << "Left wheel velocity: " << segway.left_wheel_velocity << " m/s" << std::endl;
  out << "Right wheel velocity: " << segway.right_wheel_velocity << " m/s" << std::endl;
  out << "Yaw rate: " << segway.yaw_rate << " degrees/s" << std::endl;
  out << "Servo frames: " << segway.servo_frames << " frames/s" << std::endl;
  out << "Left wheel displacement: " << segway.left_wheel_displ << " m" << std::endl;
  out << "Right wheel displacement: " << segway.right_wheel_displ << " m" << std::endl;
  out << "Forward displacement: " << segway.forward_displ << " m" << std::endl;
  out << "Yaw displacement: " << segway.yaw_displ << " rev" << std::endl;
  out << "Left motor torque: " << segway.left_torque << " Nm" << std::endl;
  out << "Right motor torque: " << segway.right_torque << " Nm" << std::endl;
  if(segway.mode==tractor)
    out << "Operation mode: tractor" << std::endl;
  else if(segway.mode==balance)
    out << "Operation mode: balance" << std::endl;
  else 
    out << "Operation mode: power down" << std::endl;
  if(segway.hardware_mode==tractor)
    out << "Hardware operation mode: tractor" << std::endl;
  else if(segway.hardware_mode==balance)
    out << "Hardware operation mode: balance" << std::endl;
  else 
    out << "Hardware operation mode: power down" << std::endl;
  if(segway.gain_schedule==light)
    out << "Gain schedule: light" << std::endl;
  else if(segway.gain_schedule==tall)
    out << "Gain schedule: tall" << std::endl;
  else 
    out << "Gain schedule: heavy" << std::endl;
  out << "UI battery voltage: " << segway.ui_battery << " V" << std::endl;
  out << "Powerbase battery voltage: " << segway.powerbase_battery << " V" << std::endl;
  segway.access_status.exit();

  return out;
}
