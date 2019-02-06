/**********Bu Kod el emegi göz nurudur.********/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <TimerOne.h>
MPU6050 mpu;



// uncomment "Bluetooth" if you want to see the actual pid values 
#define Bluetooth

// uncomment "robot" if you want to see the actual pid values 
//#define ROBOT


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;






int STBY = 9; //standby

//Motor A
int PWMA = 6; //Speed control
int AIN1 = 8; //Direction
int AIN2 = 7; //Direction

//Motor B
int PWMB = 11; //Speed control
int BIN1 = 10; //Direction
int BIN2 = 12; //Direction










int  cikis_max=0;  int  cikis_min=0;  
int max_toplam_dizi,max_toplam_dizi2,max_Dhata,max_Dhata2=0;

float toplam_dizi=0, toplam_dizi2=0, Dhata=0, Dhata_2=0;
int  indis=0, pwm_degis=0, teksefer=0, pwm_cikis=0;
float dizi_integral[4], dizi_turev[4];
int diziler_out[4]={1,2,3,0};

int16_t ax, ay, az;
int16_t gx, gy, gz;

  int fifo_over1=50, fifo_over2=50, basta_bekleme=400;

  uint32_t son_sure;

  //MODİFİYE EDİLEN DEGİŞKENLER BURDA
   // int eksi_sinir=-8, arti_sinir=8;
    int aci_eksi_sinir=-2, aci_arti_sinir=2;
    int aci_max_eksi_sinir=-30, aci_max_arti_sinir=30;
    float giris, cikis, cikis2,cikis3, set_noktasi=1, tplm_hata, onck_hata,  p=1, i=0, d=0.0;  
     // p i d    1.2 *t/L   2l  0.5l  
     int initial_speed=0;  //86 da başarı saglandı  



  
  int kararlilik1=0, kararlilik2=0, kararlilik3=0;
float tune =0;
float tune2 =0;
float tune3 =0;


  int sira=0, tasima=0;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

   unsigned long timebegun=0;
    unsigned long timeend=0;
   int digit_1000;
   int digit_100;
   int digit_10;
   int digit_1;
   int sign_of_number;
   char isaret;  //sign of data 
char my_array[5];
void parse_to_digits(int number);  

   int first_time=0;;

 float iTerm = 0;
  
float ki, kp, kd;
 float dt;
 float angle_error_old=0;

    
void dmpDataReady() {
    mpuInterrupt = true;
}



void setup() {
settings();
    



//kp=35;ki=1;kd=0;
 Timer1.initialize(15000); // timer 10 ms kuruldu
 dt=15;
 Timer1.attachInterrupt( timerIsr ); // attach the service routine here


// pinMode(3,OUTPUT);
//    pinMode(4,OUTPUT);
//    pinMode(5,OUTPUT);
//    pinMode(6, OUTPUT);
//    pinMode(7, OUTPUT);
//    pinMode(8, OUTPUT);




}


void loop() {


float sensorValue = analogRead(A0);//p
float sensorValue2 = analogRead(A1);//i
float sensorValue3 = analogRead(A2);//d
tune=  sensorValue*30/1023.0;   // 0 ile 1 arası degişir
tune2=  sensorValue2*10/1023.0;   // 0 ile 1 arası degişir
tune3=  sensorValue3*80/1023.0;   // 0 ile 1 arası degişir
  

//kp=1.75;ki=1.75;kd=1.75;
kp=tune; ki=tune2;  kd=tune3; 

dmp();

if(first_time==0){delay(4000); first_time=1;}
  
//    initial_speed = sensorValue *200/1023;
 //Serial.println(ypr[1]);
              
  gy=ypr[0]* 180/M_PI;    //yaw
  gx=ypr[1]* 180/M_PI;  // pitch
  gz=ypr[2]* 180/M_PI;   // roll
              //gx=ypr[1];
            //  pid_control();
  bluetooth(gx);

}



void timerIsr(){

// float sensorValue = analogRead(A0);
//               
//tune=  sensorValue*4/1023.0;   // 0 ile 1 arası degişir
//p=tune; 

pid_control();
gx=0;
}












