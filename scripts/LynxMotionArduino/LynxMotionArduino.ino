/* Jose Ceron Neto
/* Projeto de Mestrado
 *  Baseado no projete de TCC SkateBot
 *  de Vitor Izumino Sgrignoli
 */
/*******************************************/
/*      Robô Móvel LynxMotionA4WD1         */ 
/*******************************************/

#include <TimerOne.h>
#include <TimerThree.h>
#include <string.h>
#include <stdio.h>
#include <SoftwareSerial.h>
//-----------------Declaring Global Variables-----------------

#define t_interrup 10000 // time interruption rate (10ms)
#define t_PWM 50 // Period of the PWM wave set to 50us or 20kHz

/*Port Selection for the Encoder*/
#define Sel2 53 // Bit Sel at the ports 36, 41, 47 and 53 commands the reading of the MSB or LSB from each encoder
#define OE2  51 // Bit OE at the ports  34, 39, 45 and 51 activates the reading mode for each encoder
#define RST2 49 // bit RST at the ports 32, 37, 43 and 49 resets the counter of the encoder
#define Sel1 47 
#define OE1  45 
#define RST1 43

#define Sel4 41
#define OE4  39
#define RST4 37
#define Sel3 36
#define OE3  34
#define RST3 32

/* Encoder Reading Ports*/
#define port52 52 //D7 //Reading ports
#define port50 50 //D6
#define port48 48 //D5
#define port46 46 //D4
#define port44 44 //D9
#define port42 42 //D2
#define port40 40 //D1
#define port38 38 //D0
boolean TimerFlag = LOW;

/*Encoder Reading Variables*/
int EncoderPulses1 = 0; //Encoder 1 counting variable
int EncoderPulses2 = 0; //Encoder 2 counting variable
int EncoderPulses3 = 0; //Encoder 3 counting variable
int EncoderPulses4 = 0; //Encoder 4 counting variable
float Speed1       = 0; //rad/s
float Speed2       = 0; //rad/s
float Speed3       = 0; //rad/s
float Speed4       = 0; //rad/s

/*Motor Driving Variables*/
//Set the ports to drive the motor
#define SBT1         30		//Sabertooth driver 1 - controls Motors - HIGH activate command/LOW deactivate command
//#define SBT2 	     33		//Sabertooth driver 2 - controls Right side Motors - HIGH activate command/LOW deactivate command
#define SABER_TX_PIN 18		//Motor drive command

#define SABER_RX_PIN   19 	// Not in use - just for initialization
#define SABER_BAUDRATE 9600 // Baudrate defined by pins in drivers
SoftwareSerial SaberSerial = SoftwareSerial( SABER_RX_PIN, SABER_TX_PIN );

//Sets the variables to drive the motor
int SetPWM = 0; // from 0 to 255

//set M1 Motors command range
#define SABER_MOTORA_FULL_FORWARD  127
#define SABER_MOTORA_FULL_REVERSE  1
//set M2 Motors command range
#define SABER_MOTORB_FULL_FORWARD  255
#define SABER_MOTORB_FULL_REVERSE  128
/***************************************
For each Sabertooth:
- 0   STOP all motors

- 1   Set Motor A FULL Reverse
- 64  Set Motor A SOFT Stop
- 127 Set Motor A FULL Forward

- 128 Set Motor B FULL Reverse
- 192 Set Motor B SOFT Stop
- 255 Set Motor B FULL Forward 
***************************************/

/*Control Variables*/
const float T  = 0.01;

const float Kc1 = 3.1;
const float Td1 = 0.013;
const float Ti1 = 0.055;
const float Ki1 = T*Kc1/Ti1;
const float Kd1 = Kc1*Td1/T;


const float Kc2 = 3.1;
const float Td2 = 0.013;
const float Ti2 = 0.055;
const float Ki2 = T*Kc2/Ti2;
const float Kd2 = Kc2*Td2/T;

//const float Ki = 1;
//const float Kd = 1;

float prevError1 = 0;
float prevError2 = 0;
float prev1Speed1 = 0;
float prev1Speed2 = 0;
float prev2Speed1 = 0;
float prev2Speed2 = 0;
//SetPoint
float SetPoint1 = 0; // from 0 to 27rad/s
float SetPoint2 = 0;

int uM1 = 0;
int uM2 = 0;
int cont = 0;

/*Communication Variables*/
#define TAM_MSG  15
char inputChar[TAM_MSG];
union Data {
  int i;
  char b[2];
  };

//---------------------Timer Interruption---------------------

void TimeInterurpt(){ //Faz a EncoderReading dos Encoders 1 a 4
  EncoderPulses1 = EncoderReading(OE1, Sel1, RST1);
  EncoderPulses2 = EncoderReading(OE2, Sel2, RST2);
  //EncoderPulses3 = EncoderReading(OE3, Sel3, RST3);
  //EncoderPulses4 = EncoderReading(OE4, Sel4, RST4);
  TimerFlag = HIGH;
}
//--------------------Serial Interruption---------------------

void serialEvent(){ 
  getMsg ();
}

//-----------------------Ports Setup--------------------------

void setup(){
  // put your setup code here, to run once:
  Serial.begin(57600);  
  Timer1.initialize(t_interrup);
  Timer3.initialize(t_PWM);
  Timer1.attachInterrupt(TimeInterurpt);
  
  pinMode(RST1,OUTPUT); // Reset1 Pin
  pinMode(RST2,OUTPUT); // Reset2 Pin
  //pinMode(RST3,OUTPUT); // Reset3 Pin
  //pinMode(RST4,OUTPUT); // Reset4 Pin
  
  pinMode(OE1,OUTPUT);  // OE1 Pin
  pinMode(OE2,OUTPUT);  // OE2 Pin
  //pinMode(OE3,OUTPUT);  // OE3 Pin
  //pinMode(OE4,OUTPUT);  // OE4 Pin
  
  pinMode(Sel1,OUTPUT); // Sel1 Pin
  pinMode(Sel2,OUTPUT); // Sel2 Pin
  //pinMode(Sel3,OUTPUT); // Sel3 Pin
  //pinMode(Sel4,OUTPUT); // Sel4 Pin
  
  pinMode(port38,INPUT);// Bit 00
  pinMode(port40,INPUT);// Bit 01
  pinMode(port42,INPUT);// Bit 02
  pinMode(port44,INPUT);// Bit 03
  pinMode(port46,INPUT);// Bit 04
  pinMode(port48,INPUT);// Bit 05
  pinMode(port50,INPUT);// Bit 06
  pinMode(port52,INPUT);// Bit 07
  
  //Timer3.pwm(MotorPWM1,SetPWM1);
  //Timer3.pwm(MotorPWM2,SetPWM2);
  pinMode( SABER_TX_PIN, OUTPUT );
  pinMode( SBT1, OUTPUT );
  //pinMode( SBT2, OUTPUT );
  SaberSerial.begin( SABER_BAUDRATE ); 
  delay( 2000 ); // Time to initialize Sabertooth
  
  digitalWrite(OE1,HIGH); 
  digitalWrite(Sel1,HIGH);
  digitalWrite(OE2,HIGH); 
  digitalWrite(Sel2,HIGH);
  //digitalWrite(OE3,HIGH); 
  //digitalWrite(Sel3,HIGH);
  //digitalWrite(OE4,HIGH); 
  //digitalWrite(Sel4,HIGH);

}



//------------------------Loop Section------------------------

void loop(){
   // put your main code here, to run repeatedly:
   if (TimerFlag == HIGH){
    //Velocidade [rad/s] = EncoderPulses*(2*pi/(30*400*0.01))
    //(reducao*ppr*tempo-leitura)
    Speed1 = EncoderPulses1*(-0.052359878);
    Speed2 = EncoderPulses2*(+0.052359878);
  
    //
    ControlePID1 ();
    ControlePID2 ();
    //uM1 = 1000;
    //uM2 = -50;
    SetMotor ();
    //cont++;

    //if(cont == 1000){
      //uM2 = 100;
      //
      //SetPoint1 = 0; // from 0 to 27rad/s
      //SetPoint2 = 0;
    //}
    TimerFlag = LOW;

    //Serial.print("Speed:");
    //Serial.print(Speed2);
    //Serial.print(" , ");
    //Serial.print("uM:");
    //Serial.print(uM2);
    //Serial.print(" , ");
    //Serial.print("SetPoint:");
    //Serial.println(SetPoint2);
    
    outputMsg (Speed1,Speed2);
    
    if(digitalRead(22)) {digitalWrite(22,LOW);}
    else {digitalWrite(22,HIGH);}
    }
  }
//------------------------------------------------------------
//                         Functions
//------------------------------------------------------------

//----------------------Encoder Reading-----------------------

int EncoderReading(int OE, int Sel, int RST){
  digitalWrite(OE,LOW); //Initiate Reading
  digitalWrite(Sel,LOW);//Reading MSB 
  boolean bit15 = digitalRead(port52);
  boolean bit14 = digitalRead(port50);
  boolean bit13 = digitalRead(port48);
  boolean bit12 = digitalRead(port46);
  boolean bit11 = digitalRead(port44);
  boolean bit10 = digitalRead(port42);
  boolean bit09 = digitalRead(port40);
  boolean bit08 = digitalRead(port38);

  digitalWrite(Sel,HIGH); //Reading LSB 
  boolean bit07 = digitalRead(port52);   
  boolean bit06 = digitalRead(port50);
  boolean bit05 = digitalRead(port48);
  boolean bit04 = digitalRead(port46);
  boolean bit03 = digitalRead(port44);
  boolean bit02 = digitalRead(port42);
  boolean bit01 = digitalRead(port40);
  boolean bit00 = digitalRead(port38);
  
  digitalWrite(OE,HIGH); // Finish Reading
  digitalWrite(RST,LOW); // Resets Counter
  digitalWrite(RST,HIGH);// Restores Counter Reading 

  int EncoderPulses = 0  ; // Reset/create couter variable EncoderReading 
  
  // concatenação dos bits 
  EncoderPulses = EncoderPulses | (bit00 << 0 ); // bitXX goes to the position XX and than it is sumed up to the previous value ( Or operation )
  EncoderPulses = EncoderPulses | (bit01 << 1 );
  EncoderPulses = EncoderPulses | (bit02 << 2 );
  EncoderPulses = EncoderPulses | (bit03 << 3 );
  EncoderPulses = EncoderPulses | (bit04 << 4 );
  EncoderPulses = EncoderPulses | (bit05 << 5 );
  EncoderPulses = EncoderPulses | (bit06 << 6 );
  EncoderPulses = EncoderPulses | (bit07 << 7 );
  EncoderPulses = EncoderPulses | (bit08 << 8 );
  EncoderPulses = EncoderPulses | (bit09 << 9 );
  EncoderPulses = EncoderPulses | (bit10 << 10 );
  EncoderPulses = EncoderPulses | (bit11 << 11 );
  EncoderPulses = EncoderPulses | (bit12 << 12 );
  EncoderPulses = EncoderPulses | (bit13 << 13 );
  EncoderPulses = EncoderPulses | (bit14 << 14 );
  EncoderPulses = EncoderPulses | (bit15 << 15 );

  if (EncoderPulses <= 32768){ // The bottom half of the counter is positive (the counter size is 2 to the power of 16)
   //EncoderPulses = EncoderPulses;
  }
  else {
    EncoderPulses = (EncoderPulses - 65535); // The top half of the counter is positive
  }
  return EncoderPulses;
}

//-----------------------Motor Driving------------------------

void SetMotor (){
  // Logic to convert the output of the controler to a Sabertooth reference.
  if (uM1 == 0){
	digitalWrite(SBT1, HIGH);
	SaberSerial.write(byte(64));
  //SaberSerial.write(byte(0));
	digitalWrite(SBT1, LOW);
  }
  //if (uM1 == 0){
  //  uM1 = 1;
  //}
  
  if (uM1 != 0) {
    digitalWrite(SBT1, HIGH);
	  if(uM1 > 100){
		  SaberSerial.write(byte(SABER_MOTORA_FULL_FORWARD));
		  //SaberSerial.write(byte(SABER_MOTORB_FULL_FORWARD));
		  digitalWrite(SBT1, LOW);
	  }
	  else if (uM1 < -100){
		  SaberSerial.write(byte(SABER_MOTORA_FULL_REVERSE));
		  //SaberSerial.write(byte(SABER_MOTORA_FULL_REVERSE));
		  digitalWrite(SBT1, LOW);
	}
	else {
		unsigned char SpeedMotorA = map(uM1,-100,100,SABER_MOTORA_FULL_REVERSE,SABER_MOTORA_FULL_FORWARD);
    //Serial.println(SpeedMotorA);
		//unsigned char SpeedMotorB = map(uM1,-100,100,SABER_MOTORB_FULL_REVERSE,SABER_MOTORB_FULL_FORWARD);
		SaberSerial.write(byte(SpeedMotorA));
		//SaberSerial.write(byte(SpeedMotorB));
		digitalWrite(SBT1, LOW);
	  }
  }
  
	if (uM2 == 0){
	digitalWrite(SBT1, HIGH);
	SaberSerial.write(byte(192));
  //SaberSerial.write(byte(0));
	digitalWrite(SBT1, LOW);
  }
  //if (uM2 == 0){
  //  uM2 = 1;
  //}
  
  
  if (uM2 != 0) {
    digitalWrite(SBT1, HIGH);
	if(uM2 >= 100){
		//SaberSerial.write(byte(SABER_MOTORA_FULL_FORWARD));
		SaberSerial.write(byte(SABER_MOTORB_FULL_FORWARD));
		digitalWrite(SBT1, LOW);
	}
	else if (uM2 <= -100){
		//SaberSerial.write(byte(SABER_MOTORA_FULL_REVERSE));
		SaberSerial.write(byte(SABER_MOTORB_FULL_REVERSE));
		digitalWrite(SBT1, LOW);
	}
	else {
		//unsigned char SpeedMotorA = map(uM2,-100,100,SABER_MOTORA_FULL_REVERSE,SABER_MOTORA_FULL_FORWARD);
		unsigned char SpeedMotorB = map(uM2,-100,100,SABER_MOTORB_FULL_REVERSE,SABER_MOTORB_FULL_FORWARD);
		//SaberSerial.write(byte(SpeedMotorA));
		SaberSerial.write(byte(SpeedMotorB));
		digitalWrite(SBT1, LOW);
	}
  }
}


//---------------------PID Controller M1----------------------

void ControlePID1 () {  
  float error = SetPoint1-Speed1;

  float deltaU = Kc1*(error - prevError1) + error*Ki1 - Kd1*(Speed1 - 2*prev1Speed1 + prev2Speed1);
  deltaU = floor(deltaU);
  uM1 = uM1 + deltaU;
 
  prevError1 = error;
  prev1Speed1 = Speed1;
  prev2Speed1 = prev1Speed1;
}

//---------------------PID Controller M2----------------------

void ControlePID2 () {
  float error = SetPoint2-Speed2;
  
  float deltaU = Kc2*(error - prevError2) + error*Ki2 - Kd2*(Speed2 - 2*prev1Speed2 + prev2Speed2);
  deltaU = floor(deltaU);
  uM2 = (uM2 + deltaU);

  prevError2 = error;
  prev1Speed2 = Speed2;
  prev2Speed2 = prev1Speed2;
}

//--------------------Send Output Message---------------------

void outputMsg (float Speed1, float Speed2){
  char outputChar[8];
  Data PrintVel1, PrintVel2;
  
  PrintVel1.i = (int)(Speed1*100);
  PrintVel2.i = (int)(Speed2*100);

  outputChar[0] = '$';
  outputChar[1] = PrintVel1.b[0]; 
  outputChar[2] = PrintVel1.b[1];
  outputChar[3] = PrintVel2.b[0];
  outputChar[4] = PrintVel2.b[1];
  outputChar[5] = '#';
  outputChar[6] = '\n';
  outputChar[7] = '\0';
  
  Serial.write(outputChar,8);
}

//---------------------Get Input Message----------------------

void getMsg (){
  char inputChar[6];
  Serial.readBytes(inputChar,1);
  Data getVel1, getVel2;

  if (inputChar[0] == '$'){
    Serial.readBytes(inputChar,5);
      if (inputChar[4] == '#'){
      getVel1.b[0] = inputChar[0];
      getVel1.b[1] = inputChar[1];
      getVel2.b[0] = inputChar[2];
      getVel2.b[1] = inputChar[3];
    
      SetPoint1 = (float)getVel1.i/100;
      SetPoint2 = (float)getVel2.i/100;
    }
  }
  else{
    //clearSTR();
  } 
}

//---------------------Clear Input Buffer---------------------

void clearSTR (){
  int cnt = 0;
    for (int i = 0; i < TAM_MSG; i++)
    {
      inputChar[i] = ' ';  
    }
}
