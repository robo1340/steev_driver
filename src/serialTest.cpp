 /***
  * This example expects the serial port has a loopback on it.
  *
  * Alternatively, you could use an Arduino:
  *
  * <pre>
  *  void setup() {
  *    Serial.begin(<insert your baudrate here>);
  *  }
  *
  *  void loop() {
  *    if (Serial.available()) {
  *      Serial.write(Serial.read());
  *    }
  *  }
  * </pre>
  */
 #include <string>
 #include <iostream>
 #include <cstdio>
 // OS Specific sleep
 #ifdef _WIN32
 #include <windows.h>
 #else
 #include <unistd.h>
 #endif
 #include "serial/serial.h"
 using std::string;
 using std::exception;
 using std::cout;
 using std::cin;
 using std::cerr;
 using std::endl;
 using std::vector;
 void my_sleep(unsigned long milliseconds) {
 #ifdef _WIN32
       Sleep(milliseconds); // 100 ms
 #else
       usleep(milliseconds*1000); // 100 ms
 #endif
 }
 void enumerate_ports()
 {
         vector<serial::PortInfo> devices_found = serial::list_ports();
         vector<serial::PortInfo>::iterator iter = devices_found.begin();
         while( iter != devices_found.end() )
         {
                 serial::PortInfo device = *iter++;
                 printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
      device.hardware_id.c_str() );
         }
 }
 void print_usage()
 {
         cerr << "Usage: test_serial {-e|<serial port address>} ";
     cerr << "<baudrate> [test string]" << endl;
 }
 int run(int argc, char **argv)
 {
   string port = "/dev/ttyUSB0";
   unsigned long baud = 38400;
   serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
   cout << "Is the serial port open?";
   if(my_serial.isOpen())
     cout << " Yes." << endl;
   else
     cout << " No." << endl;
   //note that endl denotes the end of line character '\n'

   string response;
   string request;
   int inputBufferSize;

   while (1) {
	cout << ">>";
	cin >> request;
	request = request + "\r\n";
	my_serial.write(request);
	usleep(500000);//sleep for 500 milliseconds

        if ((inputBufferSize = my_serial.available()) > 0){
		response = my_serial.read(inputBufferSize);
		cout << response << endl;
	}

   }
    // size_t bytes_wrote = my_serial.write(test_string);
    // string result = my_serial.read(test_string.length()+1);
   // my_serial.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
   // cout << "Timeout == 250ms, asking for 1 more byte than written." << endl;
 }
 int main(int argc, char **argv) {
   try {
     return run(argc, argv);
   } catch (exception &e) {
     cerr << "Unhandled Exception: " << e.what() << endl;
   }
 }
