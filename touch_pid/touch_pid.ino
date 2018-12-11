//Define touchscreen connections
#include <Servo.h>

#define Xres 160
#define Yres 120
float xSum, ySum, xAvg, yAvg;
float lastY = 0;
float lastX = 0;
float noUpdateY = false;
float noUpdateX = false;
boolean redo = false;
/* Iterators used in computing running average */
float x,y; //real-time position of ball
int i,j,k;
int l = 0;
int o = 0;
int m = 0;
int n = 0;
int p = 0;
/*******************************************/

const float sqrt3 = 1.732;

const float xMin = 46.03;
const float yMin = 25.30;

const float yGoal = 0;
const float xGoal = 0;

/*Servos*/
Servo platformServo[6];
int pins[] = {3,5,6,9,10,11};
float arg[6];
/********/
float error[3];
bool noPrint = false;

/* Time Variables */
float now, deltaT, lastT;
float compT = 10;
/*****************/

/*PID Terms*/
float kp[] = {0.035,0.035,0.035};
float kd[] = {0.15,0.15,0.15};
float ki[] = {0.00005,0.00005,0.00005};
double ka[3] = {0.00,0.00,0.00};

double ITerm[3] = {0,0,0};

double lastError[3] = {0,0,0};
double lastLastError[3] = {0,0,0};
double lastDVal[3] = {0,0,0};
double lastLastDVal[3] = {0,0,0};

double lastPropTerm = 0;
boolean notNormal = true;
boolean firstCalc = true;
/*********/

/*output terms*/
float output[3];
float outputMedianArray[5];
float lastOutput[3] = {0,0,0};
/************/



/********/
double DValArray[3][6];
double DTerm[3];
double DValSum[3];
boolean sample = false;
boolean firstSample = true;
/********/
void setup() {
  pinMode(A2,INPUT_PULLUP);
  Serial.begin(9600);
    Serial.begin(9600);
  
  for(i = 0; i < 6; ++i){
    platformServo[i].attach(pins[i]);
    platformServo[i].write(90);
  }  
}

void loop() {
  now = millis();
  deltaT = now - lastT;
  if(deltaT >= 2.0){
    //take mean
    controlPID();
    //then compute args
    computeArgs();
    writeServos();
  
    lastT = now;
  }
  
}

void controlPID(){ 
    for(j = 0; j <3; ++j){
    getCoordinates();
    if(!noPrint){
    computeError(j);
    }
  }

  for(k = 0; k < 3; ++k){
    if(!noPrint){
      computePID(k);
    }
  
  }
  
  if(!noPrint){
    computeArgs();
  }
  for(j = 0; j > 5; ++j ){
    limitOutput(j);
  }
    
}

void computePID(int i){
  
    double propTerm = error[i]*kp[i];
    double DVal = (error[i] - lastError[i]);
    DTerm[i] = kd[i]*(((5*DVal) + (3*lastDVal[i]) + (2*lastLastDVal[i]))/2.0);
    ITerm[i] += (ki[i] * error[i]);
    double ATerm = ka[i] * (((DVal - lastDVal[i]) + (lastDVal[i] - lastLastDVal[i]))/2.0);
    
   if(firstSample){
      DValSum[i] = DTerm[i] + DValSum[i];
    }
    
    if(i == 0 && firstSample){
      DValArray[i][m] = DTerm[i];
      ++m;
    }
    else if(i == 1 && firstSample){
      DValArray[i][n] = DTerm[i];
      ++n;
    }
    else if(i == 2 && firstSample){
      DValArray[i][p] = DTerm[i];
      ++p;
    }

    if(l == 17 && firstSample){
      DTerm[i] = DValSum[i]/18;
      sample = true;
      firstSample = false;
    }
    if(sample){
     for(o = 0; o < 5; ++o){
       DValArray[i][o]  =  DValArray[i][o + 1];
       
     }
       DValArray[i][5] = DTerm[i];
       DTerm[i] = (DValArray[i][0] +   DValArray[i][1] + DValArray[i][2] + DValArray[i][3] +   DValArray[i][4] +  DValArray[i][5] )/6.0;
    }
    
    output[i] = propTerm + DTerm[i] + ITerm[i] + ATerm;
    
    lastError[i] = error[i];
    lastLastDVal[i] = lastDVal[i];
    lastDVal[i] = DVal;
    ++l;
  
}

void computeError(int i){
  
    if(i == 0){ //error projected onto lower left basis.
                //if error > in y direcion, then lift platform up.
  
      error[0] = 4*(0.5*(x - xGoal) + sqrt3*0.5*( yGoal -  y));

    }
    if(i == 1){ //erro onto lower right basis.

     error[1] = 4*(0.5*(x - xGoal) + sqrt3*0.5*( y - yGoal));
    
    }
    if(i == 2){

      error[2] =  4*(xGoal - x);
    
    }  
    
}


void computeArgs(){
  arg[5] = 90 - output[0];
  arg[0] = 90 + output[0];

  arg[3] = 90 + output[1];
  arg[4] = 90 - output[1];

  arg[1] = 90 + output[2];
  arg[2] = 90 - output[2];
  
}

void limitOutput(int i ){
 
  if(output[i] > 181){
    output[i] = 180;
    Serial.println("ARGUMENT ON SERVO TOO LARGE! LIMITING IT!");
  }
  else if(output[i] < -1){
    output[i] = 0;
    Serial.println("ARGUMENT ON SERVO TOO SMALL! LIMITING IT!");
  }

  if(ITerm[i] > 181) ITerm[i] = 180;
  else if(ITerm[i] < -1 ) ITerm[i] = 90;
}

void writeServos(){
  /*Basis 1*/
  platformServo[5].write(arg[3]);
  platformServo[0].write(arg[4]);

  /*Basis 2*/
  platformServo[3].write(arg[5]);
  platformServo[4].write(arg[0]);

  /*Basis 3*/
  platformServo[1].write(arg[1]);
  platformServo[2].write(arg[2]);

}
void getCoordinates(){
    //let's meaa\sure x axis touch position
  
    //Set Left Electrodes to GND
    ySum = 0;
    xSum = 0;
    
    for(i = 1; i < 3; ++i){
   pinMode(A2,INPUT);
   pinMode(A3,INPUT);  
   digitalWrite(A3,LOW);
   pinMode(A0,OUTPUT);
   digitalWrite(A0,HIGH);
   pinMode(A1,OUTPUT);
   digitalWrite(A1,LOW);
    
      xSum = float(analogRead(A2))/(1024/Xres) + xSum;
      x = -(float(xSum)/2-83);
    }                       //Measure SENSE pin (Voltage divider)


     //let's meaasure y axis touch position
   for(i = 1; i < 3; ++i){
     pinMode(A0,INPUT);
   pinMode(A1,INPUT);
   digitalWrite(A1,LOW);
   pinMode(A2,OUTPUT);
   digitalWrite(A2,HIGH);
   pinMode(A3,OUTPUT);
   digitalWrite(A3,LOW);
      ySum  = float(analogRead(A0))/(1024/Yres) + ySum;
      y = (float(ySum)/2-65);
   }

      Serial.print(x);
      Serial.print(" y = ");
      Serial.println(y);
      lastY = y;
      lastX = x;
      noUpdateY = false;
      noUpdateX = false;
      noPrint = false;
      
     
      noUpdateY = false;
      noUpdateX = false;
      noPrint = false;
      
  
    firstCalc = false;
}
