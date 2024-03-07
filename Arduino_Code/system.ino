
#include <Servo.h>
#include <math.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 11
// Control pins
int Joint1ControlPin = A1;
int Joint2ControlPin = A2;
int Joint3ControlPin = A3;

// Control values
int Joint1Control = 512; // middle value between 0 and 1024
int Joint2Control = 512; // middle value between 0 and 1024
int Joint3Control = 512; // middle value between 0 and 1024
// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;
// Starting gripper position based on figure 6
float x = 0;         // Choose x position
float y = 0.17;      // Choose y position
float z = 0.093;     // Choose z position


// Variables
float L1 = 0;       // Joint 1 link length
float L2 = 0.093;   // Joint 2 link length
float L3 = 0.17;    // Joint 3 link length
float theta1[4];    // Holds two sets of two joint 1 angles
float theta2[4];    // Holds four values for the joint 2 angle
float theta3[4];    // Holds two sets of two joint 3 angles
float angles[3];    // Holds the final three joint angles to be written to the servo motors
float K;   // Used for calculating theta3
float K1;           // Used for calculating theta2
float K2;           // Used for calculating theta2
float r;            // Used for calculating theta2
float y2;           // Used for calculating theta2
float x2;           // Used for calculating theta2
float Pi = 3.14159; // Used for converting between radians and degrees
int i;              // Used in for loops
float Joint1Angle;
float Joint2Angle;
float Joint3Angle;  // Joint angles to be written to the servo motors
int count = 0;

int GripperOpen = 60; // Open gripper; Need to tune value
int GripperClose = 180; // Close gripper; Need to tune value
// Joint Angle Offsets
int Joint1Offset = -10; // Your value may be different
int Joint2Offset = 0; // Your value may be different
int Joint3Offset = 0; // Your value may be different

int j;
int a;
float t;

float x0 = 0;
float xf = -0.1515;
float y0 = 0.17;
float yf = 0.1806;
float z0 = 0.093;
float zf = 0.0658;
float tf = 5;

float xa0 = x0;
float xa1 = 0;
float xa2 = 3*(xf-x0)/(tf*tf);
float xa3 = -2*(xf-x0)/(tf*tf*tf);

float ya0 = y0;
float ya1 = 0;
float ya2 = 3*(yf-y0)/(tf*tf);
float ya3 = -2*(yf-y0)/(tf*tf*tf);

float za0 = z0;
float za1 = 0;
float za2 = 3*(zf-z0)/(tf*tf);
float za3 = -2*(zf-z0)/(tf*tf*tf);

void setup()        // For manual set up of X,Y,Z positions
{
    Serial.begin(9600);
    Joint1.attach(Joint1Pin);
    Joint2.attach(Joint2Pin);
    Joint3.attach(Joint3Pin);
    Gripper.attach(GripperPin);
}


void loop()             // For changing gripper position with the potentiometers      
{
     for (a = 0; a < 2; a++)
    {
      for (j = 0; j < 50; j++)
      {
        if ( a == 0)
        {
          t = t+tf/50;
        }
        else
        {
          t = t-tf/50;
        }
        x = xa0 + xa1*t + xa2*t*t + xa3*t*t*t;
        y = ya0 + ya1*t + ya2*t*t + ya3*t*t*t;
        z = za0 + za1*t + za2*t*t + za3*t*t*t;

        Serial.print("j = ");
        Serial.print(j);
        Serial.print(" ,t = ");
        Serial.print(t);
        Serial.print(" ,x = ");
        Serial.print(x);
        Serial.print(" ,y = ");
        Serial.print(y);
        Serial.print(" ,z = ");
        Serial.print(z);
        Serial.print("\n");
        
        theta1[0] = atan2(y,x) - atan2(0,+1);   // Two sets of two joint 1 angles
        theta1[1] = atan2(y,x) - atan2(0,+1);
        theta1[2] = atan2(y,x) - atan2(0,-1);
        theta1[3] = atan2(y,x) - atan2(0,-1);
        K = (pow(x,2)+pow(y,2)+pow(z,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3);
        K = min(1,K);   // Rounds K down to 1 in case the value of K is slightly above 1
        theta3[0] = atan2(+sqrt(1-pow(K,2)),K); // Two sets of two joint 3 angles
        theta3[1] = atan2(-sqrt(1-pow(K,2)),K);
        theta3[2] = atan2(+sqrt(1-pow(K,2)),K);
        theta3[3] = atan2(-sqrt(1-pow(K,2)),K);
        for (i = 0; i < 4; i = i + 1)           // Four joint 2 angles
        {
            K1 = z;
            K2 = x*cos(theta1[i])+y*sin(theta1[i]);
            r = sqrt(pow(K1,2)+pow(K2,2));
            y2 = L3*cos(theta3[i])+L2;
            x2 = L3*sin(theta3[i]);
            theta2[i] = atan2(y2/r,x2/r) - atan2(K2,K1);
        }
        for (i = 0; i < 4; i = i + 1)
        {
            theta1[i] = 180*theta1[i]/Pi;       // Convert to degrees from radians
            theta2[i] = 180*theta2[i]/Pi;
            theta3[i] = -180*theta3[i]/Pi;
        
            if ( theta1[i] < 0)
            {
                theta1[i] = theta1[i] + 360;    // Make the angles positive, in the range 0 to 360, by adding 360 degrees if negative
            }
            if ( theta2[i] < 0)
            {
                theta2[i] = theta2[i] + 360;
            }
            if ( theta3[i] < 0)
            {
                theta3[i] = theta3[i] + 360;
            }
            if ( 0 <= theta1[i] && theta1[i] < 190)         // Joint 1 can rotate roughly 190 degrees
            {
                if ( 0 <= theta2[i] && theta2[i] < 140)     // Joint 2 can rotate roughly 140 degrees
                {
                    if ( 0 <= theta3[i] && theta3[i] < 160) // Joint 3 can rotate roughly 160 degrees
                    {
                        angles[0] = theta1[i];  // If the angles are suitable for the servo motos
                        angles[1] = theta2[i];  // Store the set of three angles
                        angles[2] = theta3[i];
                    } 
                }  
            }
        }
    
        Joint1Angle = angles[0];
        Joint2Angle = angles[1];
        Joint3Angle = angles[2];
        Joint1.write(Joint1Angle+Joint1Offset);     //Write to servo motors
        Joint2.write(Joint2Angle+Joint2Offset); 
        Joint3.write(180-Joint3Angle+Joint3Offset);
        delay(100);
      }
      delay(10000);
    }

}
