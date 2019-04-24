#ifndef IMU_UDP_H
#define IMU_UDP_H

#ifdef __cplusplus
#include "UDPsocket.h"
#include <math.h>
#include <string>
#include <Eigen/Dense>
#include <fstream>

class ImuUdp {
public:
  ImuUdp() {
    myfile.open("data.dat");
    myfile_euler.open("data1.dat");
    myfile_theta.open("data2.dat");
    rcv.open();
    rcv.bind(PortNo);
    sd.open();
  	sd.broadcast(true);
  };
  //for convert binary expression to IEEE 754 float
  typedef union {

      float f;
      struct
      {
          // Order is important.
          // Here the members of the union data structure
          // use the same memory (32 bits).
          // The ordering is taken
          // from the LSB to the MSB.
          unsigned int mantissa : 23;
          unsigned int exponent : 8;
          unsigned int sign : 1;
      } raw;
  } myfloat;

  float imuListen();
  void imuSend(std::string str);
  void imuSendChar(char* ch);

private:
  //socket and ip for receiving
  UDPsocket::IPv4 ipaddr_rcv;//("192.168.0.14", 9750);
	UDPsocket rcv;
  //quaternion received (int format)
  uint32_t quat_rcv[4];
  //transfered quaternion
  float quat[4];
  //receiving checking flag
  int received = 1;

  //socket and ip for sending
  UDPsocket sd;

  // Function to convert a binary array
  // to the corresponding integer
  unsigned int convertToInt(int* arr, int low, int high);
  //for convert int to binary expression
  void intToBinDigit(uint32_t in, int count, int* out);
  //port number
  static constexpr uint16_t PortNo = 9751;

  //for data record, delettte later
  int count = 0;
  float dt = 0.002;
  std::ofstream myfile;
  std::ofstream myfile_euler;
  std::ofstream myfile_theta;

  //for theta initialization
  float out_init;
  bool initialized = false;

};
#endif

//wrapper functions for c implementation
#ifdef __cplusplus
extern "C" {
#endif

  void* ImuUdp_c();
  void imuSend_c(void *imu, char* ch);
  float imuListen(void *imu);

#ifdef __cplusplus
}
#endif

#endif //end of imu_udp
