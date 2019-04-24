#include <imu_udp.h>

float ImuUdp::imuListen() {
  std::string data;
  //for intermediate result in the quaternion transformation
  int digit[32];
  //if receiving error
  if (received < 0) {
    fprintf(stderr, "recv(): failed\n");
    return -10;
  }

  received = rcv.recv(data, ipaddr_rcv, quat_rcv);
    //convert to quaternion format
  for (int k=0; k<4; k++) {
    intToBinDigit(quat_rcv[k], 32, digit);
    myfloat var;
    unsigned f = convertToInt(digit, 9, 31);
    var.raw.mantissa = f;
    f = convertToInt(digit, 1, 8);
    var.raw.exponent = f;
    var.raw.sign = digit[0];
    quat[k] = var.f;
    std::cout << quat[k] << " ";
  }
  std::cout << std::endl;
    //convert to euler angle
  Eigen::Quaternionf quater(quat[0], quat[1], quat[2], quat[3]); // w, x, y, z
  auto euler = quater.toRotationMatrix().eulerAngles(0, 1, 2);
  std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;
  myfile << (double)dt*count << " " << euler[0] << " " << euler[1] << " " << euler[2] << std::endl;
  myfile_euler << (double)dt*count << " " << quat[0] <<" " << quat[1] << " " << quat[2] << " " << quat[3] << std::endl;

  //for theta
  double output = 0;
  if(!initialized)
    out_init = acos(quat[0])*2;
  else
    output = acos(quat[0])*2 - out_init;
  myfile_theta << (double)dt*count << " " << output << std::endl;
  count ++;
  return output;
  //fprintf(stderr, "recv(): failed\n");
}

void ImuUdp::imuSend(std::string str) {
  UDPsocket::IPv4 ipaddr_sd("192.168.0.14", 9750);
  for (uint8_t i = 0; i < 5; ++i)  //send for 5 times to make sure imu received
  {
    int sent = sd.send(str, ipaddr_sd);
    if (sent<0)
    {
      fprintf(stderr, "send(): failed (REQ)\n");
    }
  }
}

void ImuUdp::imuSendChar(char* ch) {
  std::string str(ch);
  ImuUdp::imuSend(str);
}

unsigned int ImuUdp::convertToInt(int* arr, int low, int high) {
    unsigned f = 0, i;
    for (i = high; i >= low; i--) {
        f = f + arr[i] * pow(2, high - i);
    }
    return f;
}


//for convert int to binary expression
void ImuUdp::intToBinDigit(uint32_t in, int count, int* out) {
    /* assert: count <= sizeof(int)*CHAR_BIT */
    unsigned int mask = 1U << (count-1);
    int i;
    for (i = 0; i < count; i++) {
        out[i] = (in & mask) ? 1 : 0;
        in <<= 1;
    }
}

//wrapper functions for c implementation
void* ImuUdp_c() {
   ImuUdp *imucomm( new ImuUdp);
   return( reinterpret_cast< void* >( imucomm ) );
}

void imuSend_c( void *imucomm, char* ch) {
   reinterpret_cast< ImuUdp* >( imucomm )->imuSendChar(ch);
}

float imuListen_c( void *imucomm) {
   float k = reinterpret_cast< ImuUdp* >( imucomm )->imuListen();
   return k;
}
