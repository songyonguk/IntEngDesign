#include <Servo.h>
Servo myservo;

#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10


#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

float raw_dist,dist_cali;
float a, b; // unit: mm


void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  
// initialize serial port
  Serial.begin(57600);

  a = 70;
  b = 300;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(raw_dist > 156 && raw_dist <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(20);

  if(dist_cali < 255) {
    myservo.writeMicroseconds(1800);
  }
  else if(dist_cali > 255) {
   myservo.writeMicroseconds(900);
  }
//else{
 // myservo.writeMicroseconds((float)(10*dist_cali-1291));
 //

}
