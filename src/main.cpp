#include <Arduino.h>

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "Adafruit_MCP23017.h"
#include <ezOutput.h>

#include <Encoder.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS

unsigned long currentMillis;

// Gestion des réservoires
struct Tank{
  int TankID;
  bool remplissage;
  bool vidange;
  uint16_t LEDVanne; // Num de la LED de vidange
  uint8_t BPMax; // BP de test capteur haut
  uint8_t BPVanne; // BP Vanne de vidage
  uint16_t LEDhaut; // LED capteur max
  uint16_t ArrLedLevel[5]; // Led de niveau
  int Level=-1; // Niveau actuel
  int MaxLevel; // Niveau max
  bool ExternVide; //demande de vidage externe
};

Tank TankArr[8]; //array de tank
uint8_t SesorTankManuVidePin[8]={8,9,10,11,12,13,14,15};
uint16_t LedVideTank[8]={77,78,79,80,81,82,83,84};
int EzOutLevalMax[8]={8,19,18,7,9,10,11,12};
int TankMaxFill=5;
unsigned long previousMillisTank =0;
unsigned long intervalTankFill=1300; //Vitesse de remplissage
bool IntervalTankFillRuned=false;

int IstNmbrTankFillRunde;
int SollNmbrTankFillRunde=3;
int tankradomID;
bool RandomVide;

//structure pour le moteur
struct Motor{
  int MotorID;
  uint8_t State; //0 off, 1 starting, 2 On, 3 stopping
  uint16_t LedID;
  unsigned long TimeStart;
  unsigned long PrevMillisMotor;
  uint8_t MotorPin;
  bool MotorPinStatu;
};

//création du moteur
Motor M1;
#define PINIxMCPMotor 8 //Pin 2 SubD
#define LEDMotor 68 //LED D69+
bool MotorOn=false;

// Structur pour les vannes
struct Vanne{
  int VanneID;
  uint8_t State; //0 close, 1 openning, 2 open, 3 closeing
  uint8_t LEDNumber;
  uint16_t LedID0;
  uint16_t LedID1;
  uint16_t LedID2;
  unsigned long TimeStart;
  unsigned long PrevMillisVanne;
  uint8_t VannePin;
  bool VannePinStatu;
};

//Vanne Y1
Vanne Y1;
#define PINIxMCPY1 9 //Pin 3 SubD
#define LEDY100 67 //LED D68
#define LEDY110 65 //LED D66
#define LEDY111 66 //LED D67

// Vanne Y2
Vanne Y2;
#define PINIxMCPY2 10 //Pin 4 SUBD
#define LEDY2 64

// Vannes Y3-10
uint8_t PINIxMCPY[8]={11,12,13,14,15,0,1,2};
uint16_t LEDY0[8]={55,58,53,56,60,62,51,49};
uint16_t LEDY1[8]={54,59,52,57,61,63,50,48};
bool YActive[8]={false,false,false,false,false,false,false,false};


//Interval pour le blink des leds de flow
unsigned long previousMillisFlow =0;
unsigned long intervalFlow = 60;
bool TimmerFlowRuned=false;


//NeoPixel
#define PIN 5 // Pin where NeoPixels are connected
#define NUMPIXELS 85
// Declare our NeoPixel strip object:
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
//Couleur Flow
uint32_t PixelOff=pixels.Color(0,0,0);
uint32_t PixelVert=pixels.Color(0,255,0);
uint32_t PixelRouge=pixels.Color(255,0,0);
uint32_t PixelEau=pixels.Color(0,0,255);
uint32_t PixelProd=pixels.Color(167,12,139);
uint32_t PixelIOAct=pixels.Color(250,200,17);
uint32_t FlowColor;

//Encoder
Encoder Enco(4, 6);
int8_t EncoSel=1;
int8_t EncoStepFactor=3;


//MCP
Adafruit_MCP23017 mcpTankIO; // S'occupe de boutons sur la partie Tank
Adafruit_MCP23017 mcpSUBD; // s'occupe des IO pour la Sub-d IN
Adafruit_MCP23017 mcpMCU; // s'occupe de la partie erreur et choix du programme

// Lecture des input MCPMCU
uint8_t MCUProg=0;
uint8_t MCUErr=0;


int Flowcunter=0;

//LED Flow pompe
uint16_t LEDFlowPompe[3]={71,72,73};

//LED Flow Y10
uint16_t LEDFlowY10[3]={74,75,76};


//LED Tank
int TankSoll=0;
int TankIst=0;


//capteurs
uint8_t SensorState=0;
bool SensorType=false; //false= pression True = Débit

// output on arduino to Sub-D 25 with ezOutput
ezOutput Out2(23);  //TankSoll bit 0
ezOutput Out3(22);  //TankSoll bit 1
ezOutput Out4(21);  //TankSoll bit 2
ezOutput Out5(20);  //Sensor pression ou débit
ezOutput Out6(8);   //B1
ezOutput Out7(19);  //B2
ezOutput Out8(18);  //B3
ezOutput Out9(7);   //B4
ezOutput Out10(9);  //B5
ezOutput Out11(10); //B6
ezOutput Out12(11); //B7
ezOutput Out13(12); //B8

ezOutput LEDInt(13);

ezOutput OutCuve[8]={Out6,Out7,Out8,Out9,Out10,Out11,Out12,Out13};



//======================================================================

void setup() {
  //MCPs
  mcpTankIO.begin();
  mcpSUBD.begin(4);
  mcpMCU.begin(6);

  pinMode(0,INPUT);

  //Start NeoPixels
  pixels.begin();
  pixels.setBrightness(12);
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(167, 12, 69));
    pixels.show();
    delay(10);
  }
  delay(100);
  pixels.clear();
  

  for (int i = 0; i < 16; i++)
  {
    mcpTankIO.pinMode(i,INPUT);//mcpTank toutes en I
    mcpTankIO.pullUp(i,HIGH);
    mcpSUBD.pinMode(i,INPUT);//mcpSubD toutes en I
  }
  for (int i = 8; i < 16; i++)//mcpMCU 8-15 en I
  {
    mcpMCU.pinMode(i,INPUT);
  }
  for (int i = 0; i < 7; i++)//mcpMCU 0-7 en I
  {
   mcpMCU.pinMode(i,OUTPUT); 
  }
  // Lecture du codeur de programme
  MCUProg= 15 & mcpMCU.readGPIO(1);
  // ajustement des valeurs 
  switch (MCUProg)
  {
  case 8:
    MCUProg=4;
    break;
  case 9:
    MCUProg=5;
    break;
  case 10:
    MCUProg=6;
    break;
  case 11:
    MCUProg=7;
    break;
  case 4:
    MCUProg=8;
    break;
  case 5:
    MCUProg=9;
    break;
  default:
    break;
  }  

  // Adaptation du comportement de la maquette 
  //au démarrage en fonction du programme selectionné
  switch (MCUProg)
  {
  case 2:
  case 3:
  case 6:
  case 7:
    for (int i = 0; i < 8; i++)
    {
      TankArr[i].Level=random(0,TankMaxFill+1);
    }
    break;
  case 4:
  case 5:
     RandomVide=true;
  case 8:
  case 9:
    RandomVide=true;
    for (int i = 0; i < 8; i++)
    {
      TankArr[i].Level=6;
    }
    break;
  default:
    break;
  }
  


  //Selection du capteur sur la sortie 5
  //si impaire débitmètre
  SensorType=MCUProg%2;
  
  //Mise en place des Tank
  for (int i = 0; i < 8; i++)
  {
    TankArr[i].TankID=i+1;
    TankArr[i].remplissage=false;
    TankArr[i].vidange=false;
    TankArr[i].MaxLevel=TankMaxFill;
    TankArr[i].LEDVanne=77+i;
    TankArr[i].BPMax=7-i;
    TankArr[i].BPVanne=8+i;
    TankArr[i].LEDhaut=40+i;

    for (int y = 0; y < 5; y++)
    {
      TankArr[i].ArrLedLevel[y]=(5*i)+y;
    }
  }
  
    //Mise à jour du niveau du tank initial   
    for (int i = 0; i < 8; i++)
    {    
    int y=0;
    while (y<=TankArr[i].Level)
    {
      pixels.setPixelColor(TankArr[i].ArrLedLevel[y],PixelProd);
      y++;
    }
    }

  //mise en place du moteur
  M1.MotorID=1;
  M1.LedID=LEDMotor;
  M1.MotorPin=PINIxMCPMotor;
  M1.State=0;
  M1.TimeStart=2500;
  M1.MotorPinStatu=false;

  //Mise en place des vannes
    //Y1
    Y1.VanneID=1;
    Y1.LEDNumber=3;
    Y1.LedID0=LEDY100;
    Y1.LedID1=LEDY110;
    Y1.LedID2=LEDY111;
    Y1.VannePin=PINIxMCPY1;
    Y1.State=0;
    Y1.TimeStart=1000;
    Y1.VannePinStatu=false;

    //Y2
    Y2.VanneID=2;
    Y2.LEDNumber=1;
    Y2.LedID0=LEDY2;
    Y2.VannePin=PINIxMCPY2;
    Y2.State=0;
    Y2.TimeStart=1800;
    Y2.VannePinStatu=false;


  Serial.begin(115200);

  LEDInt.blink(100,400);
}


//======================================================================
void loop() {


//Serial.println("Y310[0].State");
 
  //EzOutput
  LEDInt.loop();
  Out2.loop();
  Out3.loop();
  Out4.loop();
  Out5.loop();
  Out6.loop();
  Out7.loop();
  Out8.loop();
  Out9.loop();
  Out10.loop();
  Out11.loop();
  Out12.loop();
  Out13.loop();
 
  //MAJ NeoPixel
  pixels.show(); 

  // Lecture du code erreur pour la simulation de pannes
  MCUErr=(240 & mcpMCU.readGPIO(1))/16;

//=============================
//=============================
//Gestion Moteur

M1.MotorPinStatu=mcpSUBD.digitalRead(M1.MotorPin);
//M1.MotorPinStatu=!digitalRead(0);

switch (M1.State)
{
case 0: //Motor off
  pixels.setPixelColor(M1.LedID,PixelRouge);

  if (M1.MotorPinStatu)
  {
    M1.PrevMillisMotor = currentMillis;
    M1.State=1;
  }
  
  break;
case 1: //Motor starting
  if(LEDInt.getState()){
    pixels.setPixelColor(M1.LedID,PixelIOAct);
  }else{
    pixels.setPixelColor(M1.LedID,PixelOff);
  }

  if (M1.MotorPinStatu && currentMillis - M1.PrevMillisMotor > M1.TimeStart)
  {
    M1.State=2;
  }

  if (!M1.MotorPinStatu)
  {
    M1.State=3;
  }
  
  
  break;
case 2: //Motor On
  pixels.setPixelColor(M1.LedID,PixelVert);

  if (!M1.MotorPinStatu)
  {
    M1.PrevMillisMotor = currentMillis;
    M1.State=3;
  }
  
  break;
case 3: // Motor Stopping
   if(LEDInt.getState()){
    pixels.setPixelColor(M1.LedID,PixelIOAct);
  }else{
    pixels.setPixelColor(M1.LedID,PixelOff);
  }

  if (!M1.MotorPinStatu && currentMillis - M1.PrevMillisMotor > M1.TimeStart)
  {
    M1.State=0;
  }

  if (M1.MotorPinStatu)
  {
    M1.State=1;
  }
    break;
default:

  if(LEDInt.getState()){
    pixels.setPixelColor(M1.LedID,PixelRouge);
  }else{
    pixels.setPixelColor(M1.LedID,PixelOff);
  }
  break;
}

//=============================
//=============================

  //Lecture Y1
  Y1.VannePinStatu=mcpSUBD.digitalRead(PINIxMCPY1);
  
  switch (Y1.State)
  {
  case 0: //Vanne close
    pixels.setPixelColor(Y1.LedID0,PixelVert);
    pixels.setPixelColor(Y1.LedID1,PixelVert);
    pixels.setPixelColor(Y1.LedID2,PixelRouge);
    
    if (Y1.VannePinStatu)
    {
      Y1.PrevMillisVanne=currentMillis;
      Y1.State=1;
    }
  
    break;
  case 1: //Vanne openning
    if(LEDInt.getState()){
      pixels.setPixelColor(Y1.LedID0,PixelIOAct);
      pixels.setPixelColor(Y1.LedID2,PixelIOAct);
    }else{
      pixels.setPixelColor(Y1.LedID0,PixelOff);
      pixels.setPixelColor(Y1.LedID2,PixelOff);
    }

     if (Y1.VannePinStatu && currentMillis - Y1.PrevMillisVanne > Y1.TimeStart)
    {
      Y1.State=2;
    }

    if (!Y1.VannePinStatu)
    {
      Y1.State=3;
    }
      break;
  case 2: //Vanne Open
    pixels.setPixelColor(Y1.LedID0,PixelVert);
    pixels.setPixelColor(Y1.LedID1,PixelRouge);
    pixels.setPixelColor(Y1.LedID2,PixelVert);

    if  (!Y1.VannePinStatu)
    {
      Y1.PrevMillisVanne = currentMillis;
      Y1.State=3;
    }
    break;
  case 3: // Vanne closing
    if(LEDInt.getState()){
      pixels.setPixelColor(Y1.LedID0,PixelIOAct);
      pixels.setPixelColor(Y1.LedID1,PixelIOAct);
    }else{
      pixels.setPixelColor(Y1.LedID0,PixelOff);
      pixels.setPixelColor(Y1.LedID1,PixelOff);
    }

    if (!Y1.VannePinStatu && currentMillis - Y1.PrevMillisVanne > Y1.TimeStart)
    {
      Y1.State=0;
    }

    if (Y1.VannePinStatu)
    {
      Y1.State=1;
    }
    break;
  default:
    if(LEDInt.getState()){
      pixels.setPixelColor(Y1.LedID0,PixelRouge);
    }else{
      pixels.setPixelColor(Y1.LedID0,PixelOff);
    }
    break;
  }

//=============================
//=============================

  // géstion Y2
  Y2.VannePinStatu=mcpSUBD.digitalRead(PINIxMCPY2);

  
  switch (Y2.State)
  {
  case 0: //Vanne close
    pixels.setPixelColor(Y2.LedID0,PixelRouge);
    
    if (Y2.VannePinStatu)
    {
      Y2.PrevMillisVanne=currentMillis;
      Y2.State=1;
    }
  
    break;
  case 1: //Vanne openning
    if(LEDInt.getState()){
      pixels.setPixelColor(Y2.LedID0,PixelIOAct);
    }else{
      pixels.setPixelColor(Y2.LedID0,PixelOff);
    }

     if (Y2.VannePinStatu && currentMillis - Y2.PrevMillisVanne > Y2.TimeStart)
    {
      Y2.State=2;
    }

    if (!Y2.VannePinStatu)
    {
      Y2.State=3;
    }
      break;
  case 2: //Vanne Open
    pixels.setPixelColor(Y2.LedID0,PixelVert);

    if  (!Y2.VannePinStatu)
    {
      Y2.PrevMillisVanne = currentMillis;
      Y2.State=3;
    }
    break;
  case 3: // Vanne closing
    if(LEDInt.getState()){
      pixels.setPixelColor(Y2.LedID0,PixelIOAct);
    }else{
      pixels.setPixelColor(Y2.LedID0,PixelOff);
    }

    if (!Y2.VannePinStatu && currentMillis - Y2.PrevMillisVanne > Y2.TimeStart)
    {
      Y2.State=0;
    }

    if (Y2.VannePinStatu)
    {
      Y2.State=1;
    }
    break;
  default:
    if(LEDInt.getState()){
      pixels.setPixelColor(Y2.LedID0,PixelRouge);
    }else{
      pixels.setPixelColor(Y2.LedID0,PixelOff);
    }
    break;
  }

  
//=============================
//=============================
  //Lecture Y3..Y10

    //NEOPixel -Y3...Y10
  for (int i = 0; i < 8; i++)
  {
  YActive[i]=mcpSUBD.digitalRead(PINIxMCPY[i]);
   if (YActive[i])
    {
      pixels.setPixelColor(LEDY0[i], pixels.Color(255, 0, 0));
      pixels.setPixelColor(LEDY1[i], pixels.Color(0,255,0));
    }else{
      pixels.setPixelColor(LEDY0[i], pixels.Color(0,255,0));
      pixels.setPixelColor(LEDY1[i], pixels.Color(255, 0, 0));
    }
  }
    
  // Lecture de l'encoder
  int8_t newPos = Enco.read();
  if (newPos != EncoSel) {
    EncoSel = newPos;
    if (EncoSel>8*EncoStepFactor+2)
    {
      EncoSel=1;
      Enco.write(1);
    }
    if (EncoSel<1)
    {
      EncoSel=8*EncoStepFactor;
      Enco.write(8*EncoStepFactor);
    }
    
  }
//Affichage sur le 7 segment et Tank Soll
  switch (EncoSel) 
  {
  case 3: // Codeur = 1
    mcpMCU.digitalWrite(0,LOW);//A
    mcpMCU.digitalWrite(1,HIGH);//B
    mcpMCU.digitalWrite(2,HIGH);//C
    mcpMCU.digitalWrite(3,LOW);//D
    mcpMCU.digitalWrite(6,LOW);//E
    mcpMCU.digitalWrite(4,LOW);//F
    mcpMCU.digitalWrite(5,LOW);//G
    TankSoll=1;
    break;

  case 6: // Codeur = 2
    mcpMCU.digitalWrite(0,HIGH);//A
    mcpMCU.digitalWrite(1,HIGH);//B
    mcpMCU.digitalWrite(2,LOW);//C
    mcpMCU.digitalWrite(3,HIGH);//D
    mcpMCU.digitalWrite(6,HIGH);//E
    mcpMCU.digitalWrite(4,LOW);//F
    mcpMCU.digitalWrite(5,HIGH);//G
    TankSoll=2;
    break;
  case 9: // Codeur = 3
    mcpMCU.digitalWrite(0,HIGH);//A
    mcpMCU.digitalWrite(1,HIGH);//B
    mcpMCU.digitalWrite(2,HIGH);//C
    mcpMCU.digitalWrite(3,HIGH);//D
    mcpMCU.digitalWrite(6,LOW);//E
    mcpMCU.digitalWrite(4,LOW);//F
    mcpMCU.digitalWrite(5,HIGH);//G
    TankSoll=3;
    break;
  case 12: // Codeur = 4
    mcpMCU.digitalWrite(0,LOW);//A
    mcpMCU.digitalWrite(1,HIGH);//B
    mcpMCU.digitalWrite(2,HIGH);//C
    mcpMCU.digitalWrite(3,LOW);//D
    mcpMCU.digitalWrite(6,LOW);//E
    mcpMCU.digitalWrite(4,HIGH);//F
    mcpMCU.digitalWrite(5,HIGH);//
    TankSoll=4;
    break;
  case 15: // Codeur = 5 
    mcpMCU.digitalWrite(0,HIGH);//A
    mcpMCU.digitalWrite(1,LOW);//B
    mcpMCU.digitalWrite(2,HIGH);//C
    mcpMCU.digitalWrite(3,HIGH);//D
    mcpMCU.digitalWrite(6,LOW);//E
    mcpMCU.digitalWrite(4,HIGH);//F
    mcpMCU.digitalWrite(5,HIGH);//G
    TankSoll=5;
    break;
  case 18: // Codeur = 6
    mcpMCU.digitalWrite(0,HIGH);//A
    mcpMCU.digitalWrite(1,LOW);//B
    mcpMCU.digitalWrite(2,HIGH);//C
    mcpMCU.digitalWrite(3,HIGH);//D
    mcpMCU.digitalWrite(6,HIGH);//E
    mcpMCU.digitalWrite(4,HIGH);//F
    mcpMCU.digitalWrite(5,HIGH);//G
    TankSoll=6;
    break;
  case 21: // Codeur = 7
    mcpMCU.digitalWrite(0,HIGH);//A
    mcpMCU.digitalWrite(1,HIGH);//B
    mcpMCU.digitalWrite(2,HIGH);//C
    mcpMCU.digitalWrite(3,LOW);//D
    mcpMCU.digitalWrite(6,LOW);//E
    mcpMCU.digitalWrite(4,LOW);//F
    mcpMCU.digitalWrite(5,LOW);//G
    TankSoll=7;
    break;
  case 24: // Codeur = 8
    mcpMCU.digitalWrite(0,HIGH);//A
    mcpMCU.digitalWrite(1,HIGH);//B
    mcpMCU.digitalWrite(2,HIGH);//C
    mcpMCU.digitalWrite(3,HIGH);//D
    mcpMCU.digitalWrite(6,HIGH);//E
    mcpMCU.digitalWrite(4,HIGH);//F
    mcpMCU.digitalWrite(5,HIGH);//G
    TankSoll=8;
    break;

  default:
    break;
  }

//Affectation du TankSoll sur les sorties
if (!digitalRead(0))
{
switch (TankSoll)
{
case 0:
  Out2.low();
  Out3.low();
  Out4.low();
  break;
case 1:
  Out2.low();
  Out3.low();
  Out4.low();
  break;
case 2:
  Out2.high();
  Out3.low();
  Out4.low();
  break;
case 3:
  Out2.low();
  Out3.high();
  Out4.low();
  break;
case 4:
  Out2.high();
  Out3.high();
  Out4.low();
  break;
case 5:
  Out2.low();
  Out3.low();
  Out4.high();
  break;
case 6:
  Out2.high();
  Out3.low();
  Out4.high();
  break;
case 7:
  Out2.low();
  Out3.high();
  Out4.high();
  break;
case 8:
  Out2.high();
  Out3.high();
  Out4.high();
  break;
default:
  break;
}
}

// TankIst
  if (!YActive[7])
  {
    TankIst=0;
  }else{
    if (!YActive[0] && !YActive[1] && !YActive[5])
    { 
      TankIst=8;
    }
    if (!YActive[0] && !YActive[1] && YActive[5])
    {
      TankIst=7;
    }
    if (!YActive[0] && YActive[1] && !YActive[4])
    {
      TankIst=6;
    }
    if (!YActive[0] && YActive[1] && YActive[4])
    {
      TankIst=5;
    }
    if (YActive[0] && !YActive[2] && !YActive[3])
    {
      TankIst=4;
    }
    if (YActive[0] && !YActive[2] && YActive[3])
    {
      TankIst=3;
    }
    if (YActive[0] && YActive[2] && !YActive[6])
    {
      TankIst=2;
    }
    if (YActive[0] && YActive[2] && YActive[6])
    {
      TankIst=1;
    }
  }

//TankIst=TankSoll;//Pour les tests

// Choix du la couleur du Flow

    if (Y1.State==0)
    {
    FlowColor=PixelEau;
    SensorState=1;
    }if (Y1.State==2)
    {
    FlowColor=PixelProd;
    SensorState=1;
    }
  if (M1.State==2 && Y2.State==2)
  { 
    if (TankArr[TankIst-1].Level>TankArr[TankIst-1].MaxLevel && YActive[7])
    {
    FlowColor=PixelRouge;
    SensorState=2;
    }
    if (Y1.State==1||Y1.State==3)
    {
      FlowColor=(PixelProd+PixelEau)/2;
    }
  }

  if (M1.State!=2 || Y1.State==1 || Y1.State==3 || Y2.State!=2 || FlowColor==PixelRouge )
  {
    intervalFlow=100;
  }else{
    intervalFlow=55;
  }

  if (M1.State==0 ||  Y2.State==0)
  {
    FlowColor=PixelOff;
    SensorState=0;
  }
  

  currentMillis = millis();
 if(currentMillis - previousMillisFlow > intervalFlow) { 
    previousMillisFlow = currentMillis;  
    TimmerFlowRuned=true;
    }else{
    TimmerFlowRuned=false;
 }



  

//LED Flow pompe et Y10
  if (TimmerFlowRuned)
  {
    Flowcunter++;

    switch (Flowcunter)
    {
    case 1:
      pixels.setPixelColor(LEDFlowPompe[0], PixelOff);
      pixels.setPixelColor(LEDFlowPompe[1], PixelOff);
      pixels.setPixelColor(LEDFlowPompe[2], PixelOff);
        if (!YActive[7])
        {
           pixels.setPixelColor(LEDFlowY10[0], FlowColor);
           pixels.setPixelColor(LEDFlowY10[1], PixelOff);
           pixels.setPixelColor(LEDFlowY10[2], PixelOff);
        }
      break;
    
    case 2:
      pixels.setPixelColor(LEDFlowPompe[0], FlowColor);
      pixels.setPixelColor(LEDFlowPompe[1], PixelOff);
      pixels.setPixelColor(LEDFlowPompe[2], PixelOff);
       if (!YActive[7])
        {
           pixels.setPixelColor(LEDFlowY10[0], FlowColor);
           pixels.setPixelColor(LEDFlowY10[1], FlowColor);
           pixels.setPixelColor(LEDFlowY10[2], PixelOff);
        }else{
          pixels.setPixelColor(LEDFlowY10[0], PixelOff);
          pixels.setPixelColor(LEDFlowY10[1], PixelOff);
          pixels.setPixelColor(LEDFlowY10[2], PixelOff);
        }
      break;

    case 3:
      pixels.setPixelColor(LEDFlowPompe[0], FlowColor);
      pixels.setPixelColor(LEDFlowPompe[1], FlowColor);
      pixels.setPixelColor(LEDFlowPompe[2], PixelOff);
       if (!YActive[7])
        {
           pixels.setPixelColor(LEDFlowY10[0], PixelOff);
           pixels.setPixelColor(LEDFlowY10[1], FlowColor);
           pixels.setPixelColor(LEDFlowY10[2], FlowColor);
        }else{
          pixels.setPixelColor(LEDFlowY10[0], PixelOff);
          pixels.setPixelColor(LEDFlowY10[1], PixelOff);
          pixels.setPixelColor(LEDFlowY10[2], PixelOff);
        }
      break;

    case 4:
      pixels.setPixelColor(LEDFlowPompe[0], PixelOff);
      pixels.setPixelColor(LEDFlowPompe[1], FlowColor);
      pixels.setPixelColor(LEDFlowPompe[2], FlowColor);
         if (!YActive[7])
        {
           pixels.setPixelColor(LEDFlowY10[0], PixelOff);
           pixels.setPixelColor(LEDFlowY10[1], PixelOff);
           pixels.setPixelColor(LEDFlowY10[2], FlowColor);
        }else{
          pixels.setPixelColor(LEDFlowY10[0], PixelOff);
          pixels.setPixelColor(LEDFlowY10[1], PixelOff);
          pixels.setPixelColor(LEDFlowY10[2], PixelOff);
        }
      break;

    case 5:
      pixels.setPixelColor(LEDFlowPompe[0], PixelOff);
      pixels.setPixelColor(LEDFlowPompe[1], PixelOff);
      pixels.setPixelColor(LEDFlowPompe[2], FlowColor);
      Flowcunter=0;
         if (!YActive[7])
        {
           pixels.setPixelColor(LEDFlowY10[0], PixelOff);
           pixels.setPixelColor(LEDFlowY10[1], PixelOff);
           pixels.setPixelColor(LEDFlowY10[2], PixelOff);
        }
      break;    
    default:
      break;
    }
     
  }
  

// Led Tank
// MAJ des réservoires
IntervalTankFillRuned=currentMillis - previousMillisTank > intervalTankFill;
 if(IntervalTankFillRuned) { 
  previousMillisTank = currentMillis;
  IstNmbrTankFillRunde++;
 }

  if (RandomVide)
  { 
    tankradomID=random(0,8);
    SollNmbrTankFillRunde=7;
    if (IstNmbrTankFillRunde>SollNmbrTankFillRunde && TankArr[tankradomID].Level>0)
    {
     TankArr[tankradomID].ExternVide=true;
     IstNmbrTankFillRunde=0;
    }

  }


  for (int i = 0; i < 8; i++)
  {
      //BP de test capteur haut
    if (!mcpTankIO.digitalRead(TankArr[i].BPMax)|| TankArr[i].Level>TankArr[i].MaxLevel)
    {
      pixels.setPixelColor(TankArr[i].LEDhaut,PixelIOAct);
      OutCuve[i].high();

    }else
    {
      pixels.setPixelColor(TankArr[i].LEDhaut,PixelOff);
      OutCuve[i].low();
    }
    if (TankIst==TankArr[i].TankID && IntervalTankFillRuned && TankArr[i].Level<TankArr[i].MaxLevel+1 && YActive[7]&& M1.State==2 && Y2.State==2)
    {
      TankArr[i].Level++;
    }
    

      //BP de Test vanne et vidange du réservoire
    if (!mcpTankIO.digitalRead(TankArr[i].BPVanne)|| TankArr[i].ExternVide)
    {
      pixels.setPixelColor(TankArr[i].LEDVanne,PixelVert);
      TankArr[i].vidange=true;
      if (IntervalTankFillRuned && TankArr[i].Level>-1)
      {
        TankArr[i].Level=TankArr[i].Level-1;
      }
    }else
    {
      pixels.setPixelColor(TankArr[i].LEDVanne,PixelRouge);
      TankArr[i].vidange=false;
    }

    if (TankArr[i].Level<0)
    {
      TankArr[i].ExternVide=false;
    }
    

    for (int y = 0; y < 5; y++)
    {
      //vidage du réservoir
      if (pixels.getPixelColor(TankArr[i].ArrLedLevel[y])!=0)
      {
        if (TankArr[i].Level<y)
        {
          pixels.setPixelColor(TankArr[i].ArrLedLevel[y],PixelOff);
        }
        
      }
      
      //remplissage
      if (pixels.getPixelColor(TankArr[i].ArrLedLevel[y])==0)
      {
        if (TankArr[i].Level>y)
        {
          pixels.setPixelColor(TankArr[i].ArrLedLevel[y],FlowColor);
        }
      }
    }
  }


  //Gestion du capteur de pression
  if (!SensorType){
    if (!YActive[7])
    {
      SensorState=0;
    }

    if (M1.State==2 && Y2.State!=2){
    SensorState=2;
    }
  

     if (!mcpSUBD.digitalRead(4))
    {
      SensorState=1;
    }

    switch (SensorState)
    {
    case 0:
      Out5.low();
      break;

    case 1:
      Out5.high();
      break;

    case 2:
      Out5.blink(300,200);
      break;
    default:
      break;
    }

    if (Out5.getState()==1)
    {
      pixels.setPixelColor(69,PixelIOAct);
    } else{
      pixels.setPixelColor(69,PixelOff);
    }
    

  } 
  //Gesiton du débitètre
  if (SensorType){

    if (!mcpSUBD.digitalRead(5))
    {
      SensorState=3;
    }

    switch (SensorState)
    {
    case 0:
    case 2:
      Out5.low();
      break;
  
    case 1:
      Out5.blink(200,80);
      break;

    case 3:
      Out5.high();
      break;

    default:
      break;
    }

    if (Out5.getState()==1)
    {
      pixels.setPixelColor(70,PixelIOAct);
    } else{
      pixels.setPixelColor(70,PixelOff);
    }
  }


}//Fin