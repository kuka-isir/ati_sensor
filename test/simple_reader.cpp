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
  float diff(1);
  uint32_t cnt(0),n(100);
  uint32_t rdt(0),ft(0),rdt_old(0),ft_old(0);
  clock_t t(0);
  // The sensor object
  cout << "Creating sensor" << endl;
  ati::FTSensor ftsensor;
  
  cout << "Initializing sensor" << endl;
  if(! ftsensor.init("192.168.100.103"))
    return -1;

  cout << "Setting timeout to 1.0sec" << endl;
  ftsensor.setTimeout(1.0);
  
  cout << "Setting sensor bias" << endl;
  ftsensor.setBias();

  // The variable where the measurements are saved
  double measurements[6];
  cout << "Getting sensor measurements" << endl;

  t = clock();
  while(1)
  {
    
    ftsensor.getMeasurements(measurements,rdt,ft);
    
    if(ft == ft_old)
    {
      cout << "ERROR : same ft as previous" <<endl;
      break;
    }
    ft_old = ft;
    
    if(cnt >= n)
    {
      t = clock() - t;
      diff = ((float)t)/CLOCKS_PER_SEC;
	  cout <<"rdt: "<<rdt<<" ft: "<<ft<<endl;
          cout << setprecision(8)<< setw(8)<< "Fx: " <<   measurements[0] 
          << setprecision(8)<< setw(8)<< " Fy: " <<   measurements[1]  
          << setprecision(8)<< setw(8)<< " Fz: " <<   measurements[2] 
          << setprecision(8)<< setw(8)<< " Tx: " <<   measurements[3]  
          << setprecision(8)<< setw(8)<< " Ty: " <<   measurements[4]  
          << setprecision(8)<< setw(8)<< " Tz: " <<   measurements[5] 
          << setprecision(8)<< setw(8)<< endl;
      cout << cnt<< " It took "<< diff <<" ms to get "<<n<<" measurements => freq="<<n/diff<<"Hz"<<endl;
      t = clock();
      cnt=0;
    }

    cnt++;
  }
  return 0;
}