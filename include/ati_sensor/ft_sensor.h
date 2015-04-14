// ISIR 2015 Antoine Hoarau <hoarau.robotics@gmail.com>

// standard and socket related libraries
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <fstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <sstream>

namespace ati{
static const std::string default_ip = "192.168.100.103";
static const int current_calibration=-1;
// Structure for the sensor response
typedef struct response_struct {
	uint32_t rdt_sequence;
	uint32_t ft_sequence;
	uint32_t status;
	int32_t Fx;
	int32_t Fy;
	int32_t Fz;
	int32_t Tx;
	int32_t Ty;
	int32_t Tz;
	uint32_t cpt;
	uint32_t cpf;
} response_s;

typedef struct command_struct{
  static const uint16_t command_header = 0x1234;
  uint16_t command;
  uint32_t sample_count;
  static const uint16_t STOP = 0x0000;
  static const uint16_t REALTIME = 0x0002;
  static const uint16_t BUFFERED = 0x0003;
  static const uint16_t MULTIUNIT = 0x0004;
  static const uint16_t RESET_THRESHOLD_LATCH = 0x0041;
  static const uint16_t SET_SOFWARE_BIAS = 0x0042;
  static const int DEFAULT_PORT = 49152;
} command_s;

class FTSensor{
public:
  // Constructor
  FTSensor();
  ~FTSensor();
  
  // Initialization, reading parameters from XML files, etc..
  bool init(std::string ip, int calibration_index = ati::current_calibration, uint16_t cmd = ati::command_s::REALTIME);
  
  // GET functions
  // Read parameters
  const double getCountsperForce(){return resp_.cpf;};
  const double getCountsperTorque(){return resp_.cpt;};
  // Read sensor values
  template<typename T>
  void getMeasurements(T measurements[6])
  {
    doComm();
    
    measurements[0]=static_cast<T>( resp_.Fx ) / static_cast<T>(resp_.cpf);
    measurements[1]=static_cast<T>( resp_.Fy ) / static_cast<T>(resp_.cpf);
    measurements[2]=static_cast<T>( resp_.Fz ) / static_cast<T>(resp_.cpf);
  
    measurements[3]=static_cast<T>( resp_.Tx ) / static_cast<T>(resp_.cpt);
    measurements[4]=static_cast<T>( resp_.Ty ) / static_cast<T>(resp_.cpt);
    measurements[5]=static_cast<T>( resp_.Tz ) / static_cast<T>(resp_.cpt);
  }
  template<typename T>
  void getMeasurements(T measurements[6],uint32_t& rdt_sequence)
  {
    getMeasurements<T>(measurements);
    rdt_sequence = resp_.rdt_sequence;
  }
  template<typename T>
  void getMeasurements(T measurements[6],uint32_t& rdt_sequence,uint32_t& ft_sequence)
  {
    getMeasurements<T>(measurements,rdt_sequence);
    ft_sequence = resp_.ft_sequence;
  }
  // Write the ip of the sensor
  const std::string getIP(){return this->ip;}
  const int getPort(){return this->port;}
  // Set the zero of the sensor	
  void setBias();
  void setTimeout(float sec);
  bool isInitialized();
protected:
  // Socket info
  bool parseCalibrationData();
  bool startRealTimeStreaming(uint32_t sample_count=1);
  bool startBufferedStreaming(uint32_t sample_count=100);
  bool startMultiUnitStreaming(uint32_t sample_count=100);
  bool resetThresholdLatch();
  bool setSoftwareBias();
  bool stopStreaming();
  bool startStreaming();
  bool openSocket();
  int closeSocket();
  void setCommand(uint16_t cmd);
  void setSampleCount(uint32_t sample_count);
  bool sendCommand();
  bool sendCommand(uint16_t cmd);
  bool getResponse();
  void doComm();
  std::string ip;
  int port;
  int calibration_index;
  int socketHandle_;     
  struct sockaddr_in addr_;  
  struct hostent *hePtr_;

  // Communication protocol
  response_s resp_;
  command_s cmd_;
  unsigned char request_[8];    
  unsigned char response_[36];
  bool initialized_;
  bool timeout_set_;
  struct timeval timeval_;

};
}
