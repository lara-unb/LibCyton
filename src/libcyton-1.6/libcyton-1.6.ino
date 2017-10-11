#include <unwind-cxx.h>
#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <utility.h>
#include <sstream>
#include <string>
#include <vector>

#include <ros.h>
#include <std_msgs/Int8.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>


ros::NodeHandle  nh;
std_msgs::Int8 cython_state;
std_msgs::Float32MultiArray joints_feedback;



ros::Publisher cython("cython", &cython_state);
ros::Publisher cython_feedback("feedback", &joints_feedback);


using namespace std;

// Minimo e Maximo em Volts
float min_array[] = {1.91, 0.55, 1.79, 2.66, 1.85, 1.81, 1.91, 1.84};
float max_array[] = {0.56, 2.52, 0.55, 0.72, 0.55, 0.47, 0.50, 0.70};

// Minimo e Maximo do Servo
float min_array2[] = {2330, 2320, 2350, 800, 2370, 2300, 2370, 2150};
float max_array2[] = {720, 920, 730, 2100, 610, 600, 580, 800};
float min_degree[] = {-90,-86, -90, -83, -90, -90,-90, -90};
float max_degree[] = {90, 65, 90, 62, 90, 90, 90, 90};

vector <float> angles_feedback;
vector <float> angles_ref;

float value = 0.0;
float setPointAngle[] = {0,0,0,0,0,0,0,0};



int servo[8] = {6,1,2,3,4,5,0,7}; // corrige leitura para que a 6 esteja mapeada na porta 0.
int duration;
int servo_is_on = 1;


class PID{
public:
  
  float error;
  float sample;
  float lastSample;
  float kP, kI, kD;      
  float P, I, D;
  float pid;
  
  float setPoint;
  float lastProcess;
  
  PID(float _kP, float _kI, float _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  
  void addNewSample(float _sample){
    sample = _sample;
  }
  
  void setSetPoint(float _setPoint){
    setPoint = _setPoint;
  }
  
  float process(){
    // Implementação P ID
    error = setPoint - sample;
    //Serial.println(setPoint);
    //Serial.print(sample);
    //Serial.print(error);
    //Serial.print(" ");
    float deltaTime = (millis() - lastProcess) / 1000.0;
    lastProcess = millis();
    
    //P
    P = error * kP;
    
    //I
    I = I + (error * kI) * deltaTime;
    
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    
    // Soma tudo
    pid = P + I + D;
    //Serial.println(pid);
    return pid;
  }
};

PID servoPID0(-0.12, 0, 0);
PID servoPID1(-0.12, 0, 0);
PID servoPID2(-0.12, 0, 0);
PID servoPID3(-0.12, 0, 0);
PID servoPID4(-0.12, 0, 0);
PID servoPID5(-0.12, 0, 0);
PID servoPID6(-0.12, 0, 0);
PID servoPID7(-0.12, 0, 0);


vector <float> Feedback();

void setCythonPose( const std_msgs::Int8MultiArray& pose){

  for (int i = 0; i < 8; i++){
    setPointAngle[i] = pose.data[i];
  } 
  
  nh.spinOnce();
}

void debug_mode( const std_msgs::Int8& debug){
  
  Serial2.println("#7 L #6 L #5 L #4 L #3 L #2 L #1 L #0 L T5500 <cd>");
  servo_is_on = 0;
  delay(550);
  if (debug.data == 1){
    servo_is_on = 1;
    Serial2.println("#7 P1500 #6 P1500 #5 P1500 #4 P1500 #3 P1500 #2 P1500 #1 P1500 #0 P1500 T2500 <cd>");
    delay(2500);
  }
  
}

ros::Subscriber<std_msgs::Int8MultiArray> subCytonPose("pose", &setCythonPose );
ros::Subscriber<std_msgs::Int8> subDebugMode("debug", &debug_mode );

void setup() {
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(subCytonPose);
  nh.subscribe(subDebugMode);
  nh.advertise(cython);
  nh.advertise(cython_feedback);
  nh.spinOnce();

  //joints_feedback.layout.dim_length = 1;
  joints_feedback.data_length = 8;
  
  Serial2.begin(9600);
  Serial2.println("#7 P1500 #6 P1500 #5 P1500 #4 P1500 #3 P1500 #2 P1500 #1 P1500 #0 P1500 T2500 <cd>");
  //Serial2.println("#7 L #6 L #5 L #4 L #3 L #2 L #1 L #0 L T5500 <cd>");
  delay(2500);
  //Serial1.begin(19200);
   //analogReference(EXTERNAL);

   
}

void loop() {
  //testPID();
  
  angles_feedback = Feedback();
  joints_feedback.data = &angles_feedback[0];
  cython_feedback.publish(&joints_feedback);
  PIDControl(angles_ref, angles_feedback);
  nh.spinOnce();

}

vector <float> Feedback(){
  vector <float> angles;
  
  for (int i = 0; i < 8; i++){
    value = analogRead(servo[i])*(5.0/1023.0);
    value = mapfloat(value, min_array[i], max_array[i], min_degree[i], max_degree[i]);
    angles.push_back(value);
    
    //Serial.print(value);
    //Serial.print(" ");
  }
  //Serial.println(" ");
  return angles;
}

/*
void update_feedback(vector <float> angles_feedback){ 

  for (int i = 0; i < 8; i++){
    joints_feedback.data[i] = 1.0;
    Serial.println(joints_feedback.data[i]);
     
  } 
  
}*/
void move_servo(vector <int> & power, vector <int> mov_time, int delay_time){
   /*for (int i = 0; i < 7; i++){
      Serial.println(power[i]);
    }*/
   for(int iservo = 0; iservo < 8; ++iservo){
       Serial2.print("#");
       Serial2.print(iservo);
       Serial2.print(" P");
       Serial2.print(power[iservo]);
       
   }
   Serial2.print(" T");
   Serial2.print(delay_time);
   Serial2.print(" ");
   Serial2.println("<cr>"); 
   
   cython_state.data = 1;
   cython.publish(&cython_state);
   nh.spinOnce();
   delay(delay_time);

   cython_state.data = 0;
   cython.publish(&cython_state);
   nh.spinOnce();
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
 
}    

void PIDControl(vector <float> angles_float, vector <float> feedback){ 
  vector <float> pidOut;
  vector <int> power_pidOut; 
  
  float pidOut0, pidOut1;  
  

  servoPID0.addNewSample(feedback[0]);servoPID1.addNewSample(feedback[1]);servoPID2.addNewSample(feedback[2]);
  servoPID3.addNewSample(feedback[3]);servoPID4.addNewSample(feedback[4]);servoPID5.addNewSample(feedback[5]);
  servoPID6.addNewSample(feedback[6]);servoPID6.addNewSample(feedback[7]);
  
  // Setpoint
  servoPID0.setSetPoint(setPointAngle[0]);servoPID1.setSetPoint(setPointAngle[1]);servoPID2.setSetPoint(setPointAngle[2]);
  servoPID3.setSetPoint(setPointAngle[3]);servoPID4.setSetPoint(setPointAngle[4]);servoPID5.setSetPoint(setPointAngle[5]);
  servoPID6.setSetPoint(setPointAngle[6]);servoPID6.setSetPoint(setPointAngle[7]);
  
  
  // Process
  pidOut.push_back(servoPID0.process());pidOut.push_back(servoPID1.process());pidOut.push_back(servoPID2.process());
  pidOut.push_back(servoPID3.process());pidOut.push_back(servoPID4.process());pidOut.push_back(servoPID5.process());
  pidOut.push_back(servoPID6.process());pidOut.push_back(servoPID7.process());
  Serial.println(" ");
  // Compensation
  duration = 1000;

  vector <int> mov_times;
   for (int i = 0; i < 8; i++){
      mov_times.push_back(duration);
    }
  
  for (int i = 0; i < 8; i++){
      power_pidOut[i] = int(mapfloat(int(setPointAngle[i] + pidOut[i]), min_degree[i], max_degree[i], min_array2[i], max_array2[i]));
      //Serial.println(power_pidOut[i]);
    }

    /*for (int i = 0; i < 7; i++){
      Serial.println(power_pidOut[i]);
    }*/
  //Serial.println(pidOut);
  
  /*for (int i = 0; i < 7; i++){
      Serial.println(power_pidOut[i]);
    }*/
  if(servo_is_on == 1){
    move_servo(power_pidOut, mov_times, duration);
  }  
  nh.spinOnce();
  
}



