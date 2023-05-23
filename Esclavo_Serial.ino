const int dirPin = 12;
const int enPin = 13;
String cad,cad1,cad2,cad3; // for incoming serial data
int dir,en,pos;
void setup() {
  Serial.begin(74880); // opens serial port, sets data rate to 9600 bps
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
}

void loop() {
  // send data only when you receive ddata:
  if (Serial.available() > 0) {
    // read the incoming byte:
    cad = Serial.readString();
    pos = cad.indexOf(',');
    cad1 = cad.substring(0,pos);
    cad2 = cad.substring(pos+1,pos+2);
    dir = cad1.toInt();
    en = cad2.toInt();
  } 
    if (en == 1){
      digitalWrite(enPin,HIGH);
    }
    if(dir == 1){
      digitalWrite(dirPin,HIGH);
    }
    if(dir == 0){
      digitalWrite(dirPin,LOW);
    }
    if (en==0){
      digitalWrite(enPin,LOW);
    }
}
