int pulPin=12;
int dirPin=14;  
int enPin=27;  
int steps = 0; 
String receivedCommand;
bool runallowed = false;

void setup() {
  Serial.begin(74880); //initialize the serial transmition at 74880 bauds
  pinMode(pulPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
}

void loop() {
  if (runallowed == true )
  {
       if(steps>0){
          controlMotor1(1, steps);
       }
       else{
          controlMotor1(0, steps*-1);
       }
  }
  else{
    checkSerial();
  }
}

void controlMotor1(bool dir, int stepM){ 
    int i=0;
    digitalWrite(dirPin,dir);
    digitalWrite(enPin,HIGH);
   while(i<stepM){ 
      digitalWrite(pulPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(pulPin, LOW);
      delayMicroseconds(500);
      i++;
    }
    digitalWrite(enPin,LOW);
    runallowed = false;
}

void checkSerial() //method for receiving the commands
{ 
  if (Serial.available() > 0) //if something comes
  {
    receivedCommand = Serial.readString();
    steps = receivedCommand.toInt();
  if(steps!=0){
  runallowed = true; 
  }
}
}
