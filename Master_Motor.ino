//Declaramos los pines que iran del arduino al driver 
//verdes a pines
//azules a 3.3
const int stepPin = 12;
const int dirPin = 14;
const int enPin = 27 ;
const int dirres = 33;
const int enres =  32;
const int Times = 400;
int pasos = 0;
String cad,cad1,cad2,cad3;
int pos,dir,en;
void setup() {
  // Declaramos entradas y salidas 
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  pinMode(dirres,INPUT);
  pinMode(enres,INPUT);
  digitalWrite(enPin, HIGH);
  digitalWrite(dirPin, HIGH);  
}

void loop() {
  en=digitalRead(enres);
  dir=digitalRead(dirres);
  for (int i=0; i < 2;i++){
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(Times); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(Times); 
  }
  if (en==1){
    digitalWrite(enPin,HIGH);
  }
  if (en==0) {
    digitalWrite(enPin,LOW);
  }
  if (dir==1){
    digitalWrite(dirPin,HIGH);
  }
  if (dir==0) {
    digitalWrite(dirPin,LOW);
  }
}
