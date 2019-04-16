#include <chrono>
#include <string>
#include <thread>
//using namespace std::literals;

#include "UDPsocket.h"
#include <math.h>
#include <iostream>

static constexpr uint16_t PortNo = 9751;

//********************************************************************************************
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

// Function to convert a binary array
// to the corresponding integer
unsigned int convertToInt(int* arr, int low, int high)
{
    unsigned f = 0, i;
    for (i = high; i >= low; i--) {
        f = f + arr[i] * pow(2, high - i);
    }
    return f;
}

//******************************************************************************************
//for convert int to binary expression
void int_to_bin_digit(uint32_t in, int count, int* out)
{
    /* assert: count <= sizeof(int)*CHAR_BIT */
    unsigned int mask = 1U << (count-1);
    int i;
    for (i = 0; i < count; i++) {
        out[i] = (in & mask) ? 1 : 0;
        in <<= 1;
    }
}

    //int digit[8];
    //int_to_bin_digit(40, 8, digit);


int main()
{
  UDPsocket::IPv4 ipaddr;//("192.168.0.14", 9750);
	UDPsocket ss;
	ss.open();
	//ss.connect(ipaddr);
  ss.bind(PortNo);
/*
  UDPsocket::IPv4 ipaddr_sd("192.168.0.14", 9750);
  UDPsocket cs;
	cs.open();
	cs.broadcast(true);

  //send
  for (uint8_t i = 0; i < 10; ++i)
  {
    int test = cs.send("Q"s, ipaddr_sd);
    std::cout << "result:" << test << std::endl;
    if (test<0)//UDPsocket::IPv4::Broadcast(PortNo)) < 0
    {
      fprintf(stderr, "send(): failed (REQ)\n");
    }
  }
*/

  //receive
  std::string data;
  //std::string part1, part2, part3, part4;
  uint32_t quat[] = {0, 0, 0, 0};
  int receive = 1;
  while (receive > 0) {
    std::cout << "count for receiving" << std::endl;
    receive = ss.recv(data, ipaddr, quat);
    std::cout << data.length() << std::endl;
    if (receive < 0)
    {
      fprintf(stderr, "recv(): failed\n");
    }
    else
    {
      std::cout << "value1：" << quat[0] << std::endl;
  		printf("0x%08x\n", quat[0]);
      std::cout << "value2：" << quat[1] << std::endl;
  		printf("0x%08x\n", quat[1]);
      std::cout << "value3：" << quat[2] << std::endl;
  		printf("0x%08x\n", quat[2]);
      std::cout << "value4：" << quat[3] << std::endl;
  		printf("0x%08x\n", quat[3]);

      int digit[32];
      int_to_bin_digit(quat[2], 32, digit);
      for (int coun=0;coun<32;coun++) {
        std::cout << digit[coun] << " ";
      }
      std::cout << std::endl;

 //*
      myfloat var;
      unsigned f = convertToInt(digit, 9, 31);
      var.raw.mantissa = f;
      f = convertToInt(digit, 1, 8);
      var.raw.exponent = f;
      var.raw.sign = digit[0];
      printf("The float value of the given"
           " IEEE-754 representation is : \n");
      printf("%f", var.f);


    }
  }

/*
  //send
  for (uint8_t i = 0; i < 10; ++i)
  {
    int test = cs.send("W"s, ipaddr);
    std::cout << "resultW:" << test << std::endl;
    if (test<0)//UDPsocket::IPv4::Broadcast(PortNo)) < 0
    {
      fprintf(stderr, "send(): failed (REQ)\n");
    }
  }
*/
	return 0;
}
