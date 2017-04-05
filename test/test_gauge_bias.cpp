#include <string>
#include <stdio.h> 
#include <ctime>
#include <iostream>
#include <iomanip>
// FTSensor class definition
#include "ati_sensor/ft_sensor.h"

using namespace std;

int main(int argc, char **argv) 
{
  uint32_t cnt(0),n(100);
  uint32_t rdt(0),ft(0),rdt_old(0),ft_old(0);
  clock_t t(0);
  string ip="192.168.100.103";
  // Read ip if given
  if (argc >1)
  {
    ip = argv[1];
  }

  // The sensor object
  cout << "Creating sensor" << endl;
  ati::FTSensor ftsensor;
  
  cout << "Initializing sensor" << endl;
  ftsensor.init(ip);

  cout << "Set bias (reset) and read gauge bias values" << endl;
  ftsensor.setBias();
  ftsensor.getCalibrationData();
  std::vector<int> gauge_bias = ftsensor.getGaugeBias();
  if (gauge_bias.size()!=6)
  {
    cout << "Get gauge bias failed, size of vector read back is " << gauge_bias.size() << endl;
    return -1;
  }
  cout << "Gauge bias values after biasing : " << gauge_bias[0] <<", "
                                        << gauge_bias[1] <<", "
                                        << gauge_bias[2] <<", "
                                        << gauge_bias[3] <<", "
                                        << gauge_bias[4] <<", "
                                        << gauge_bias[5] <<", " <<std::endl;
  
  cout << "Set gauge bias to specific values and read them back" << endl;
  gauge_bias.at(0) = 10;
  gauge_bias.at(1) = -1;
  gauge_bias.at(2) = 11;
  gauge_bias.at(3) = -2;
  gauge_bias.at(4) = 12;
  gauge_bias.at(5) = -3;
  if(ftsensor.setGaugeBias(gauge_bias))
  {
    ftsensor.getCalibrationData();
    std::vector<int> gauge_bias_new = ftsensor.getGaugeBias();
    cout << "Gauge bias values are : " << gauge_bias_new[0] <<" (" << gauge_bias[0] <<" was set), "
                                          << gauge_bias_new[1] <<" (" << gauge_bias[1] <<" was set), "
                                          << gauge_bias_new[2] <<" (" << gauge_bias[2] <<" was set), "
                                          << gauge_bias_new[3] <<" (" << gauge_bias[3] <<" was set), "
                                          << gauge_bias_new[4] <<" (" << gauge_bias[4] <<" was set), "
                                          << gauge_bias_new[5] <<" (" << gauge_bias[5] <<" was set), " <<std::endl;
  }


  return 0;
}
