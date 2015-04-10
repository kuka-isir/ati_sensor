// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
// Also add information on how to contact you by electronic and paper mail.

// ISIR 2015 Antoine Hoarau <hoarau.robotics@gmail.com>

#include "ati_sensor/ft_sensor.h"
// XML related libraries
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>
#include <sstream>
  // Read elements from XML file

void findElementRecusive(xmlNode * a_node,const xmlChar element_to_find[],xmlChar ret[])
{
  xmlNode *cur_node = NULL;
  xmlNode *cur_node_temp = NULL;
  int i=0;
  //xmlChar parameter_text[40];
  xmlChar parameter_comp[40];
  for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
    if (cur_node->type == XML_ELEMENT_NODE) {
	 xmlStrPrintf(parameter_comp,40,cur_node->name);
	 if(xmlStrEqual(parameter_comp, element_to_find)){
		cur_node_temp=cur_node->children;
		xmlStrPrintf(ret,40,cur_node_temp->content);
		continue;
	 }
    }
    findElementRecusive(cur_node->children,element_to_find,ret);
  }
  return;
}
using namespace ati;

FTSensor::FTSensor(std::string ip,unsigned int calibration_index)
  {
    if(ip.empty()){
        std::cerr<< "IP provided is empty, using default ip "<<default_ip<<std::endl; 
        this->ip = default_ip;
    }else
        this->ip = ip;
    
    this->port = command_s::DEFAULT_PORT;
    cmd_.command = command_s::STOP;
    cmd_.sample_count = 1;
    this->calibration_index = calibration_index;
  }

FTSensor::~FTSensor()
{
  
  if(stopStreaming() && 0 == closeSocket())
      std::cout << "Sensor shutdown sucessfully" << std::endl;
}
// Initialization read from XML file
bool FTSensor::init()
{
  return init(command_s::REALTIME);
}
bool FTSensor::startStreaming()
{
    switch(cmd_.command){
      case command_s::REALTIME:
	std::cout << "Starting realtime streaming" << std::endl;
	return startRealTimeStreaming();
      case command_s::BUFFERED:
	std::cout << "Starting buffered streaming" << std::endl;
	return startBufferedStreaming();
      case command_s::MULTIUNIT:
	std::cout << "Starting multi-unit streaming" << std::endl;
	return startMultiUnitStreaming();
      default:
	std::cout <<cmd_.command<< ": command mode not allowed"<< std::endl;
	return false;
    }
}

bool FTSensor::init(uint16_t cmd)
{
  if(openSocket())
  {
    stopStreaming(); // if previously launched
    setCommand(cmd); // Setting cmd mode
    startStreaming(); // Starting streaming
  }else{
    std::cerr << "Error during initialization" << std::endl;
    return false;
  }
}
bool FTSensor::openSocket()
{
  // create the socket
  socketHandle_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socketHandle_ == -1) {
      std::cerr << "failed to init sensor socket"<<std::endl;
      return false;
  }

  // Parse Calibration from web server
  if(!parseCalibrationData()){
    std::cerr << "Using default calibration parameters" << std::endl;
  }

  // set the socket parameters
  hePtr_ = gethostbyname(getIP().c_str());
  memcpy(&addr_.sin_addr, hePtr_->h_addr_list[0], hePtr_->h_length);
  addr_.sin_family = AF_INET;
  addr_.sin_port = htons(getPort());

  // connect
  int err = connect( socketHandle_, (struct sockaddr *)&addr_, sizeof(addr_) );
  if (err == -1) {
    std::cerr << "Could not connect to the sensor at "<<getIP()<<":"<<getPort();
    return false;
  }
  return true;
}
int FTSensor::closeSocket()
{
  int ret = close(this->socketHandle_) ;
  if(ret != 0){
    std::cerr << "Error while closing socket ("<<ret<<")"<<std::endl;
  }
  return ret;
}
bool FTSensor::parseCalibrationData()
{
    // parse the configuration 
  xmlNode *root_element = NULL;
  
  std::string index("");
  if(calibration_index != ati::current_calibration)
  {
    std::stringstream ss;
    ss << calibration_index;
    index = "?index=" + ss.str();
    std::cout << "Using calibration index "<<calibration_index<< std::endl;
  }else
      std::cout << "Using current calibration" << std::endl;
  std::string filename = "http://"+getIP()+"/netftapi2.xml"+index;

  xmlDocPtr doc = xmlReadFile(filename.c_str(), NULL, 0);
  
  if (doc != NULL)
  {
      root_element = xmlDocGetRootElement(doc);
      
      xmlChar cfgcpf[40];
      findElementRecusive(root_element,xmlCharStrdup("cfgcpf"),cfgcpf);
      std::stringstream cfgcpf_ss;
      cfgcpf_ss << cfgcpf;
      resp_.cpf = static_cast<uint32_t>(atoi(cfgcpf_ss.str().c_str()));
      
      xmlChar cfgcpt[40];
      findElementRecusive(root_element,xmlCharStrdup("cfgcpt"),cfgcpt);
      std::stringstream cfgcpt_ss;
      cfgcpt_ss << cfgcpt;
      resp_.cpt = static_cast<uint32_t>(atoi(cfgcpt_ss.str().c_str()));
      
      std::cout << "Sucessfully retrieved counts per force : "<<resp_.cpf<<std::endl;
      std::cout << "Sucessfully retrieved counts per torque : "<<resp_.cpt<<std::endl;
      xmlFreeDoc(doc);
  }else{
    resp_.cpf = 1000000;
    resp_.cpt = 1000000;
    std::cerr << "Could not parse file " << filename<<std::endl;
    std::cerr << "Using default counts per force : "<<resp_.cpf<<std::endl;
    std::cerr << "Using default counts per torque : "<<resp_.cpt<<std::endl;
  }
  xmlCleanupParser();
  return true;
}

bool FTSensor::sendCommand()
{
  return sendCommand(cmd_.command); 
}

bool FTSensor::sendCommand(uint16_t cmd)
{
  *reinterpret_cast<uint16_t*>(&request_[0]) = htons(command_s::command_header);
  *reinterpret_cast<uint16_t*>(&request_[2]) = htons(cmd); 
  *reinterpret_cast<uint16_t*>(&request_[4]) = htonl(cmd_.sample_count);
  return send(socketHandle_, request_, 8, 0 )==8;
}

bool FTSensor::getResponse()
{
  ssize_t ret = recv(socketHandle_, response_, 36, 0 );
  resp_.rdt_sequence = ntohl(*reinterpret_cast<uint32_t*>(&response_[0]));
  resp_.ft_sequence = ntohl(*reinterpret_cast<uint32_t*>(&response_[4]));
  resp_.status = ntohl(*reinterpret_cast<uint32_t*>(&response_[8]));
  resp_.Fx = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 0 * 4])));
  resp_.Fy = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 1 * 4])));
  resp_.Fz = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 2 * 4])));
  resp_.Tx = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 3 * 4])));
  resp_.Ty = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 4 * 4])));
  resp_.Tz = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 5 * 4])));
  return ret==36;
}

void FTSensor::doComm()
{
  if(!sendCommand())
    std::cerr << "Error while sending command" << std::endl;
  if(!getResponse())
    std::cerr << "Error while getting response, command:" <<cmd_.command <<std::endl;
}


void FTSensor::setBias()
{
  std::cout << "Setting bias"<<std::endl;
  this->setSoftwareBias();
}

void FTSensor::setTimeout(unsigned int sec, unsigned int usec)
{
  struct timeval tv;
  tv.tv_sec = sec;
  tv.tv_usec = usec;
  if (setsockopt(this->socketHandle_, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
      std::cerr << "Error setting timeout" << std::endl;
  }
}

bool FTSensor::resetThresholdLatch()
{
  if(-1 == sendCommand(command_s::RESET_THRESHOLD_LATCH)){
    std::cerr << "Could not start reset threshold latch" << std::endl;
      return false;
  }
  return true;
}
bool FTSensor::setSoftwareBias()
{
  stopStreaming();
  if(-1 == sendCommand(command_s::SET_SOFWARE_BIAS)){
    std::cerr << "Could not set software bias" << std::endl;
      return false;
  }
  startStreaming();
  return true;
}
bool FTSensor::stopStreaming()
{
  return sendCommand(command_s::STOP);
}

bool FTSensor::startBufferedStreaming(uint32_t sample_count)
{
  setSampleCount(sample_count);
  setCommand(command_s::BUFFERED);
  if(-1 == sendCommand()){
    std::cerr << "Could not start buffered streaming" << std::endl;
      return false;
  }
  return true;
}
bool FTSensor::startMultiUnitStreaming(uint32_t sample_count)
{
  setSampleCount(sample_count);
  setCommand(command_s::MULTIUNIT);
  if(-1 == sendCommand()){
    std::cerr << "Could not start multi-unit streaming" << std::endl;
      return false;
  }
  return true;
}
bool FTSensor::startRealTimeStreaming(uint32_t sample_count)
{
  setSampleCount(sample_count);
  setCommand(command_s::REALTIME);
  if(-1 == sendCommand()){
    std::cerr << "Could not start realtime streaming" << std::endl;
    return false;
  }
  return true;
}

void FTSensor::setCommand(uint16_t cmd)
{
  this->cmd_.command = cmd;
}

void FTSensor::setSampleCount(uint32_t sample_count)
{
  this->cmd_.sample_count = sample_count;
}

