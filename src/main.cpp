#define TYPE 5

#if TYPE == 1
//https://technologiehub.at/project-posts/micro-ros-on-esp32-tutorial/
//https://www.superdroidrobots.com/product_info/TE-183-002_Schematic.pdf
//https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
//#include <std_msgs/msg/float32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;

//std_msgs__msg__Int32 msg;
nav_msgs__msg__Odometry msg_odom;
//std_msgs__msg__Float32 msg_float;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#include <LS7366.h>
#include <SPI.h>

LS7366 LS7366_l(5);
LS7366 LS7366_r(17);

unsigned long count_l;
unsigned long count_r;

void error_loop() {
  while(1) {
    delay(100);
  }
}

/*
// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left() {
 
  static int lastCountL = 0;
  count_l = LS7366_l.read_counter();
  
  if(count_l != 0 && lastCountL != 0) {
         
    int leftTicks = (count_l - lastCountL);
 
    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    }
    else if (leftTicks < -10000) {
      leftTicks = 65535-leftTicks;
    }
    else{}
    distanceLeft = leftTicks/TICKS_PER_METER;
  }
  lastCountL = leftCount.data;
}
 
// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int16& rightCount) {
   
  static int lastCountR = 0;

  if(rightCount.data != 0 && lastCountR != 0) {
 
    int rightTicks = rightCount.data - lastCountR;
     
    if (rightTicks > 10000) {
      distanceRight = (0 - (65535 - distanceRight))/TICKS_PER_METER;
    }
    else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    else{}
    distanceRight = rightTicks/TICKS_PER_METER;
  }
  lastCountR = rightCount.data;
}
*/

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg_odom, NULL));

    //Time 
    struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
    msg_odom.header.stamp.sec = ts.tv_sec;
    msg_odom.header.stamp.nanosec = ts.tv_nsec;

    msg_odom.header.frame_id.data = "odom";
    msg_odom.child_frame_id.data = "base_link";

    msg_odom.pose.pose.orientation.x = count_r;
    msg_odom.pose.pose.orientation.y = count_l;

    
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "odom_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  //msg.data = 0;

  //Inicializa buffers
  LS7366_l.write_mode_register_0(FILTER_1 | DISABLE_INDX | SINGE_CYCLE | QUADRX4);
  LS7366_l.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_3 );
  LS7366_l.clear_counter();
  LS7366_l.clear_status_register();
  LS7366_l.write_data_register(4);

  LS7366_r.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  LS7366_r.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_3);
  LS7366_r.clear_counter();
  LS7366_r.clear_status_register();
  LS7366_r.write_data_register(4);

}

void loop() {

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  //Leitura dos Buffers
  
  //count_l = LS7366_l.read_counter();
  //count_r = LS7366_r.read_counter();

  delay(10);
}

#endif

#if TYPE == 2
#include <Arduino.h>
#include <LS7366.h>
#include <SPI.h>

LS7366 LS7366_l(5);
LS7366 LS7366_r(17);

unsigned long count_l;
unsigned long count_r;

int lastCount_l = 0;
int lastCount_r = 0;

float ticks_per_revolution = 2400.0;
float radius_l = 0.011;
float radius_r = 0.011;
float s = 0.295;

//float s_1 = 0.252;
//float s_2 = 0.281;

//float s_1 = 0.2615;
//float s_2 = 0.2615;

float phi_l;
float phi_r;

float x_b = 0;

float x_b_1 = 0;
float x_b_2 = 0;

float y_b = 0;
float theta = 0;

float theta_1 = 0;
float theta_2 = 0;

float displacemt_l = 0;
float displacemt_r = 0;

//Intervalo em 10ms de disparo do timer
float timer_ms = 10.0;

float displacement(int total_count);
void count_left();
void count_right();

float test_temp;

hw_timer_t *Timer0_Cfg = NULL;

void IRAM_ATTR Timer0_ISR()
{
  //unsigned long tempo_inicial = millis();
  count_left();
  count_right();

  //deg/s - left
  //phi_l = (lastCount_l * 360) / 24.0;
  //rad/s
  phi_l = ((lastCount_l * 360) / 24.0) * 0.01745;
  //deg/s - right
  //phi_r = (lastCount_r * 360) / 24.0;
  //rad/s
  phi_r = ((lastCount_r * 360) / 24.0) * 0.01745;

  theta += (((radius_r * phi_r) - (radius_l * phi_l)) / s) * 0.01;

  //theta_1 = (((radius_r * phi_r) - (radius_l * phi_l)) / s_1) * 0.01;
  //theta_2 = (((radius_r * phi_r) - (radius_l * phi_l)) / s_2) * 0.01;

  //theta += ((theta_1 + theta_2)/2.0);

  x_b += ((((radius_r * phi_r)/2) * cos(theta)) + (((radius_l * phi_l)/2) * cos(theta)))*0.01;
  y_b += ((((radius_r * phi_r)/2) * sin(theta)) + (((radius_l * phi_l)/2) * sin(theta)))*0.01;


  //x_b_1 += ((((radius_r * phi_r)/2) * cos(theta_1)) + (((radius_l * phi_l)/2) * cos(theta_1)))*0.01;
  //x_b_2 += ((((radius_r * phi_r)/2) * cos(theta_2)) + (((radius_l * phi_l)/2) * cos(theta_2)))*0.01;

  //displacemt_l += (radius_l * phi_l);
  //displacemt_r += (radius_r * phi_r);

}

void setup() {
  Serial.begin(115200);

  //ticks a cada 1us
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  //disparo a cada 1ms
  //timerAlarmWrite(Timer0_Cfg, 1000, true);
  //disparo a cada 10ms
  timerAlarmWrite(Timer0_Cfg, 10000, true);
  //disparo a cada 100ms
  //timerAlarmWrite(Timer0_Cfg, 100000, true);
  //disparo a cada 1000ms
  //timerAlarmWrite(Timer0_Cfg, 1000000, true);
  timerAlarmEnable(Timer0_Cfg);
    
  //Inicializa buffers
  LS7366_l.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  LS7366_l.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_2 );
  LS7366_l.clear_counter();
  LS7366_l.clear_status_register();
  LS7366_l.write_data_register(4);

  LS7366_r.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  LS7366_r.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_2);
  LS7366_r.clear_counter();
  LS7366_r.clear_status_register();
  LS7366_r.write_data_register(4);

}
void count_left() {
  count_l = LS7366_l.read_counter();
  if (count_l > 30000)
  {
    lastCount_l =  (65536 - count_l) * -1;
  }
  else
  {
    lastCount_l = count_l;
  }
  LS7366_l.clear_counter();
}

void count_right() {
  count_r = LS7366_r.read_counter();
  if (count_r > 30000)
  {
    lastCount_r =  (65536 - count_r) * -1;
  }
  else
  {
    lastCount_r = count_r;
  }
  LS7366_r.clear_counter();
}


void loop()
{

  /*
  Serial.print(lastCount_l);
  Serial.print(" - ");
  Serial.print(lastCount_r);
  Serial.print(" - ");

  Serial.print(phi_l);
  Serial.print(" - ");  
  Serial.print(phi_r);
  Serial.print(" - ");
  Serial.print(x_b);  
  Serial.print(" - ");
  Serial.println(theta);  
  */

  /*
  Serial.print(lastCount_l);
  Serial.print(" - ");
  Serial.print(lastCount_r);
  Serial.print(" - ");
  Serial.print(theta_1/0.01745);  
  Serial.print(" - ");
  Serial.print(theta_2/0.01745);  
  Serial.print(" - ");
  Serial.print(x_b_1);  
  Serial.print(" - ");
  Serial.print(x_b_2);  
  Serial.print(" - ");
  Serial.print(displacemt_l);  
  Serial.print(" - ");
  Serial.println(displacemt_r);  
  */

  Serial.print(theta/0.01745);  
  Serial.print(" - ");
  Serial.print(x_b);  
  Serial.print(" - ");
  Serial.println(y_b); 

  delay(10);
}
#endif

#if TYPE == 3



//
//    FILE: HX_calibration.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: HX711 calibration finder for offset and scale
//     URL: https://github.com/RobTillaart/HX711


#include "HX711.h"

HX711 myScale;

//Célula 1
//uint8_t dataPin = 27;
//uint8_t clockPin = 26;

// Célula 2
//uint8_t dataPin = 33;
//uint8_t clockPin = 32;

// Célula 3
//uint8_t dataPin = 13;
//uint8_t clockPin = 12;

// Célula 4
uint8_t dataPin = 14;
uint8_t clockPin = 25;

void calibrate();

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("HX711_LIB_VERSION: ");
  Serial.println(HX711_LIB_VERSION);
  Serial.println();

  myScale.begin(dataPin, clockPin);
}

void loop()
{
  calibrate();
}

void calibrate()
{
  Serial.println("\n\nCALIBRATION\n===========");
  Serial.println("remove all weight from the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();

  Serial.println("and press enter\n");
  while (Serial.available() == 0);

  Serial.println("Determine zero weight offset");
  //  average 20 measurements.
  myScale.tare(20);
  int32_t offset = myScale.get_offset();

  Serial.print("OFFSET: ");
  Serial.println(offset);
  Serial.println();


  Serial.println("place a weight on the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();

  Serial.println("enter the weight in (whole) grams and press enter");
  uint32_t weight = 0;
  while (Serial.peek() != '\n')
  {
    if (Serial.available())
    {
      char ch = Serial.read();
      if (isdigit(ch))
      {
        weight *= 10;
        weight = weight + (ch - '0');
      }
    }
  }
  Serial.print("WEIGHT: ");
  Serial.println(weight);
  myScale.calibrate_scale(weight, 20);
  float scale = myScale.get_scale();

  Serial.print("SCALE:  ");
  Serial.println(scale, 6);

  Serial.print("\nuse scale.set_offset(");
  Serial.print(offset);
  Serial.print("); and scale.set_scale(");
  Serial.print(scale, 6);
  Serial.print(");\n");
  Serial.println("in the setup of your project");

  Serial.println("\n\n");
}

#endif

#if TYPE == 4

#include "HX711.h"

HX711 scale_1;
HX711 scale_2;

HX711 scale_3;
HX711 scale_4;

//  adjust pins if needed.
uint8_t dataPin_1 = 27;
uint8_t clockPin_1 = 26;

uint8_t dataPin_2 = 33;
uint8_t clockPin_2 = 32;

uint8_t dataPin_3 = 13;
uint8_t clockPin_3 = 12;

uint8_t dataPin_4 = 14;
uint8_t clockPin_4 = 25;


volatile float f_1;
volatile float f_2;
volatile float f_3;
volatile float f_4;

const float l_1_2 = 130;
const float l_3_4 = 130;
const float l_2_3 = 83;

void setup()
{
  Serial.begin(115200);
  scale_1.begin(dataPin_1, clockPin_1);
  scale_1.set_offset(-180071);
  scale_1.set_scale(-1908.566040);
  scale_1.tare();

  scale_2.begin(dataPin_2, clockPin_2);
  scale_2.set_offset(213857);
  scale_2.set_scale(1897.985596);
  scale_2.tare();

  scale_3.begin(dataPin_3, clockPin_3);
  scale_3.set_offset(-681432);
  scale_3.set_scale(-1939.714722);
  scale_3.tare();

  scale_4.begin(dataPin_4, clockPin_4);
  scale_4.set_offset(179299);
  scale_4.set_scale(1887.274292);
  scale_4.tare();
}



void loop()
{
  //Célula 1
  //digitalWrite(slct_1, LOW);
  //scale.set_offset(-1210); 
  //scale.set_scale(-0.558716);
  f_1 = scale_1.get_units();
  /*
  Serial.print("Célula 1: ");
  Serial.print(f_1, 0);
  Serial.print(" ");
  */

  f_2 = scale_2.get_units();
  /*
  Serial.print("Célula 2: ");
  Serial.print(f_2, 0);
  Serial.print(" ");
  */

  f_3 = scale_3.get_units();
  /*
  Serial.print("Célula 3: ");
  Serial.print(f_3, 0);
  Serial.print(" ");
  */

  f_4 = scale_4.get_units();
  /*
  Serial.print("Célula 4: ");
  Serial.println(f_4, 0);
  */


  float a, b, total;

  if((f_1 + f_2) > 0)
  {
    a = (f_1 * l_1_2)/(f_1 + f_2);
  }
  else
  {
    a = 0;
  }
  
  if((f_3 + f_4) > 0)
  {
    b = (f_4 * l_3_4)/(f_3 + f_4);
  }
  else 
  {
    b = 0;
  }

  total = l_2_3 + a + b;

  /*
  Serial.print("a: ");
  Serial.print(a, 1);
  Serial.print(" ");
  Serial.print("b: ");
  Serial.print(b, 1);
  Serial.print(" ");
  Serial.print("total: ");
  Serial.print(total, 1);
  Serial.println(" ");
  */

  Serial.print(">a: ");
  Serial.println(a, 1);

  Serial.print(">b: ");
  Serial.println(b, 1);

  Serial.print(">total: ");
  Serial.println(total, 1);
}

#endif


#if TYPE == 5

//pio device monitor > serial.txt

#include "HX711.h"
#include <Arduino.h>
#include <LS7366.h>
#include <SPI.h>

#define btn 34

HX711 scale_1;
HX711 scale_2;

HX711 scale_3;
HX711 scale_4;

//  adjust pins if needed.
uint8_t dataPin_1 = 27;
uint8_t clockPin_1 = 26;

uint8_t dataPin_2 = 33;
uint8_t clockPin_2 = 32;

uint8_t dataPin_3 = 13;
uint8_t clockPin_3 = 12;

uint8_t dataPin_4 = 14;
uint8_t clockPin_4 = 25;

const float l_1_2 = 130;
const float l_3_4 = 130;
const float l_2_3 = 83;
float baseline = 0;

//Leitura dos encoders
LS7366 LS7366_l(5);
LS7366 LS7366_r(17);

unsigned long count_l;
unsigned long count_r;

int lastCount_l = 0;
int lastCount_r = 0;

float ticks_per_revolution = 2400.0;
float radius_l = 0.011;
float radius_r = 0.011;
float s = 0.295;

float phi_l;
float phi_r;

float x_b = 0;
float x_b_1 = 0;
float x_b_2 = 0;

float y_b = 0;
float theta = 0;

float theta_1 = 0;
float theta_2 = 0;

float displacemt_l = 0;
float displacemt_r = 0;

//Intervalo em 10ms de disparo do timer
float timer_ms = 10.0;

float displacement(int total_count);
void count_left();
void count_right();
void measure_distance();

hw_timer_t *Timer0_Cfg = NULL;

void IRAM_ATTR Timer0_ISR()
{
  //unsigned long tempo_inicial = millis();
  count_left();
  count_right();

  //deg/s - left
  //phi_l = (lastCount_l * 360) / 24.0;
  //rad/s
  phi_l = ((lastCount_l * 360) / 24.0) * 0.01745;
  //deg/s - right
  //phi_r = (lastCount_r * 360) / 24.0;
  //rad/s
  phi_r = ((lastCount_r * 360) / 24.0) * 0.01745;

  theta += (((radius_r * phi_r) - (radius_l * phi_l)) / s) * 0.01;

  //theta_1 = (((radius_r * phi_r) - (radius_l * phi_l)) / s_1) * 0.01;
  //theta_2 = (((radius_r * phi_r) - (radius_l * phi_l)) / s_2) * 0.01;

  //theta += ((theta_1 + theta_2)/2.0);

  x_b += ((((radius_r * phi_r)/2) * cos(theta)) + (((radius_l * phi_l)/2) * cos(theta)))*0.01;
  y_b += ((((radius_r * phi_r)/2) * sin(theta)) + (((radius_l * phi_l)/2) * sin(theta)))*0.01;


  //x_b_1 += ((((radius_r * phi_r)/2) * cos(theta_1)) + (((radius_l * phi_l)/2) * cos(theta_1)))*0.01;
  //x_b_2 += ((((radius_r * phi_r)/2) * cos(theta_2)) + (((radius_l * phi_l)/2) * cos(theta_2)))*0.01;

  //displacemt_l += (radius_l * phi_l);
  //displacemt_r += (radius_r * phi_r);

}

void setup() {
  Serial.begin(115200);
   
  //Inicializa buffers
  LS7366_l.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  LS7366_l.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_2 );
  LS7366_l.clear_counter();
  LS7366_l.clear_status_register();
  LS7366_l.write_data_register(4);

  LS7366_r.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  LS7366_r.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_2);
  LS7366_r.clear_counter();
  LS7366_r.clear_status_register();
  LS7366_r.write_data_register(4);

  scale_1.begin(dataPin_1, clockPin_1);
  scale_1.set_offset(-180071);
  scale_1.set_scale(-1908.566040);
  scale_1.tare();

  scale_2.begin(dataPin_2, clockPin_2);
  scale_2.set_offset(213857);
  scale_2.set_scale(1897.985596);
  scale_2.tare();

  scale_3.begin(dataPin_3, clockPin_3);
  scale_3.set_offset(-681432);
  scale_3.set_scale(-1939.714722);
  scale_3.tare();

  scale_4.begin(dataPin_4, clockPin_4);
  scale_4.set_offset(179299);
  scale_4.set_scale(1887.274292);
  scale_4.tare();


  pinMode(btn, INPUT_PULLDOWN);

  //ticks a cada 1us
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  //disparo a cada 1ms
  //timerAlarmWrite(Timer0_Cfg, 1000, true);
  //disparo a cada 10ms
  timerAlarmWrite(Timer0_Cfg, 10000, true);
  //disparo a cada 500ms
  //timerAlarmWrite(Timer0_Cfg, 500000, true);
  //disparo a cada 1000ms
  //timerAlarmWrite(Timer0_Cfg, 1000000, true);
  timerAlarmEnable(Timer0_Cfg);

}

void measure_distance()
{
  float f_1;
  float f_2;
  float f_3;
  float f_4;

  f_1 = scale_1.get_units();
  f_2 = scale_2.get_units();
  f_3 = scale_3.get_units();
  f_4 = scale_4.get_units();

  float a, b;
  if((f_1 + f_2) > 0)
  {
    a = (f_1 * l_1_2)/(f_1 + f_2);
  }
  else
  {
    a = 0;
  }
  
  if((f_3 + f_4) > 0)
  {
    b = (f_4 * l_3_4)/(f_3 + f_4);
  }
  else 
  {
    b = 0;
  }

  s = l_2_3 + a + b;
  s = s/1000;

}

void count_left() {
  count_l = LS7366_l.read_counter();
  if (count_l > 30000)
  {
    lastCount_l =  (65536 - count_l) * -1;
  }
  else
  {
    lastCount_l = count_l;
  }
  LS7366_l.clear_counter();
}

void count_right() {
  count_r = LS7366_r.read_counter();
  if (count_r > 30000)
  {
    lastCount_r =  (65536 - count_r) * -1;
  }
  else
  {
    lastCount_r = count_r;
  }
  LS7366_r.clear_counter();
}


void loop()
{

  /*
  Serial.print(lastCount_l);
  Serial.print(" - ");
  Serial.print(lastCount_r);
  Serial.print(" - ");

  Serial.print(phi_l);
  Serial.print(" - ");  
  Serial.print(phi_r);
  Serial.print(" - ");
  Serial.print(x_b);  
  Serial.print(" - ");
  Serial.println(theta);  
  */

  /*
  Serial.print(lastCount_l);
  Serial.print(" - ");
  Serial.print(lastCount_r);
  Serial.print(" - ");
  Serial.print(theta_1/0.01745);  
  Serial.print(" - ");
  Serial.print(theta_2/0.01745);  
  Serial.print(" - ");
  Serial.print(x_b_1);  
  Serial.print(" - ");
  Serial.print(x_b_2);  
  Serial.print(" - ");
  Serial.print(displacemt_l);  
  Serial.print(" - ");
  Serial.println(displacemt_r);  
  */

  Serial.print(s,3);
  Serial.print(" ");
  Serial.print(theta/0.01745);  
  Serial.print(" ");
  Serial.print(x_b, 4);  
  Serial.print(" ");
  Serial.println(y_b, 4); 

  delay(10);

  measure_distance();

  bool _btn = digitalRead(btn);
  if(_btn == false)
  {
    x_b = 0;
    y_b = 0;
    theta = 0;
  }


}
#endif