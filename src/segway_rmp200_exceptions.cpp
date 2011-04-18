#include "segway_rmp200_exceptions.h"
#include <string.h>
#include <stdio.h>

const std::string segway_rmp200_exception_msg="[CSegwayRMP200 class] - ";

CSegwayRMP200Exception::CSegwayRMP200Exception(const std::string& where,const std::string& error_msg,const std::string& segway_id):CException(where,segway_rmp200_exception_msg)
{
  this->error_msg+=error_msg;
  this->error_msg+=" - ";
  this->error_msg+=segway_id;
}
