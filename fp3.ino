
const int pingPinleft= 4;
const int pingPinright= 8;
int leftsensor=0;
int rightsensor=1;

void setup() {
  long duration, cm;
 Serial.begin(9600); 
 pinMode(duration,OUTPUT);
 pinMode(distance,OUTPUT);
}

void loop() {
  customDistanceFunction(0);
  customDistanceFunction(1);
  delay(100);
}

int customDistanceFunction(int sensor) {
  if (sensor==0) {
    pinMode(pingPinleft, OUTPUT);
    digitalWrite(pingPinleft, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPinleft, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPinleft, LOW);
    pinMode(pingPinleft, INPUT);
    duration = pulseIn(pingPinleft, HIGH);
    distance= duration / 29 / 2;
    Serial.print("Left",distance,"cm");
  }
  else {
    digitalWrite(pingPinright, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPinright, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPinright, LOW);
    pinMode(pingPinright, INPUT);
    duration = pulseIn(pingPinright, HIGH);
   distance= duration / 29 / 2;
    Serial.print("Right", distance, "cm");
  }
}