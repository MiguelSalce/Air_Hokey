//***************Librerias****************************
#include <SPI.h>
#include <time.h>  
#include <Wire.h>  

#include <SoftwareSerial.h>   //DFPlayer
#include "DFRobotDFPlayerMini.h" ////DFPlayer
//****************************************************

//***************Contadores***************************
#define boton1 5
#define boton2 4
int contador1 = 0;
int contador2 = 0;
int LED1 = 6;
int LED2 = 7;
//****************************************************

//*************DFPlayer**************************
SoftwareSerial DFPlayerSerial(10,11);   //Tx, Rx
DFRobotDFPlayerMini myDFPlayer;
// Audio 1 "risa", 2 "moneda", 3 "wii inicio", 4 "buu", 5 "vctoria"
//************************************************

void setup() {
  Serial.begin(9600);

  Wire.begin();

  //Parte para contadores
  pinMode(boton1, INPUT);
  pinMode(boton2, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  
  //Parte para iniciar DFPlayer
  DFPlayerSerial.begin(9600);
  myDFPlayer.begin(DFPlayerSerial);
  myDFPlayer.volume(30);   //De 0 a 30
  myDFPlayer.play(3); 
  delay(5000); 

}

void loop() {
  int estado1 =digitalRead(boton1);
  int estado2 =digitalRead(boton2);

  //Puntos Robot
  if (estado1 == HIGH){
    contador1++;
    Serial.println(contador1);
    delay(300);
    myDFPlayer.play(2);
    delay(1000);
    if(contador1 == 5){
      digitalWrite(LED1,HIGH);
      myDFPlayer.play(4);
    }
    
  }

  //Puntos Humano
  if (estado2 == HIGH){
    contador2++;
    delay(300);
    Serial.println(contador2);
    myDFPlayer.play(1);
    delay(2000);
    if(contador2 == 5){
      digitalWrite(LED1,HIGH);
      myDFPlayer.play(5);
    }
    
  }

  //GANADOR

  
}
