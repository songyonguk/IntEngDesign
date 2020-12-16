#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 // [3097] LED핀 9번으로 설정
#define PIN_SERVO 10 // [3097] 서보핀 9번으로 설정
#define PIN_IR A0 // [3097] 적외선핀 A0번으로 설정

// Framework setting
#define _DIST_TARGET 255 //[0711] 목표값
#define _DIST_MIN 100 //[0711] 측정 최솟값
#define _DIST_MAX 410 //[0711] 측정 최댓값

// Servo range
#define _DUTY_MIN 1380   // [1615] 서보 제어 펄스 폭: 최고 각도
#define _DUTY_NEU 1550  // [1615] 서보 제어 펄스 폭: 수평
#define _DUTY_MAX 2200  // [1615] 서보 제어 펄스 폭: 최저 각도

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 25

// Event periods
#define _INTERVAL_DIST 20  // [3074] 거리측정주기 (ms)
#define _INTERVAL_SERVO 20 // [3078] 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // [3078] Serial제어주기 (ms)

#define DELAY_MICROS  1500 // Filter Sample Delay by Park
#define EMA_ALPHA 0.4     // EMA alpha by Park

// PID parameters
#define _KP 2.7//[0711] 비례이득
#define _KD 42
#define _KI 0.015
/*
 * #define _KP 0.7//[0711] 비례이득
#define _KD 8
#define _KI 0.002
 */
//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;
// global variables
const float coE[] =  {-0.0000042, 0.0062855, -0.2368431, 100.1351777};
//{0.0000070, 0.0005019, 0.6784435, 60.2473829};

//{0.0000914, -0.0329004, 4.8902973, -103.8251536};
//{0.0000222, -0.0044069, 1.2405964, 38.4477060};
//{-0.0000047, 0.0047226, 0.3355967, 64.0217618};
//{0.0000075, 0.0031092, 0.0562535, 94.4963591};
//{0.0000047, 0.0061183, -0.4912427, 114.4093900};
//{-0.0000431, 0.0199730, -0.6240180, 69.3974244};
//114.4093900
// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;  // [1615] 센서가 인식한 거리/ema 필터링된 거리

float filtered_dist;
float ema_dist = 0;
float samples_num = 8;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {

  // initialize serial port
Serial.begin(57600); // [3083] serial port의 baud rate를 57600으로 초기화
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED, OUTPUT); // [3083] PIN_LED 핀을 OUTPUT으로 설정
myservo.attach(PIN_SERVO); // [3083] PIN_SERVO 핀을 servo를 쓰는 핀으로 설정
pterm = iterm = dterm = 0;
// initialize global variables

// move servo to neutral position
//myservo.writeMicroseconds(_DUTY_NEU); // [3090]
//  delay(5000);


  // initialize last sampling time
  last_sampling_time_dist = last_sampling_time_serial = 0;
  event_dist = event_serial = false;


// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0); // [3074] 서보 업데이트 1주기에 증감 가능한 duty 의 최댓값

  // [1615] 마지막 이벤트 발생 시간 초기화
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  event_dist = false;
  event_servo = false;
  event_serial = false;
}

float distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0; 
  return val;
}

float ir_distance(void){ // return value unit: mm
  float result;
  float x = distance();
  result = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  return result;
}
/*
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = (250000 * pow(volt, -1.165)-60);
  return val;}
*/
/*
 * 
 */

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 460;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}


// val = (350000 * pow(volt, -1.24)-40);  
void loop() {
/////////////////////
// Event generator //
/////////////////////
// [1615] 거리 측정 주기가 되었는지 검사 
unsigned long time_curr = millis();
if (millis() >= last_sampling_time_dist + _INTERVAL_DIST)
        event_dist = true;
    
  // [1615] 서보 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO)
      event_servo= true;
    
  // [1615] Serial 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)
      event_serial= true;

      



////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
  
    event_dist = false;
    dist_raw = ir_distance();
    //==============================================
    ema_dist = filtered_ir_distance();
    //===============================================
 

  // PID control logic
    error_curr = (ema_dist - _DIST_TARGET);// [3073] 현재 읽어들인 데이터와 기준 값의 차이
    pterm = (_KP*error_curr);// [3073] p게인 값인 kp와 error 값의 곱
    dterm = _KD*(error_curr - error_prev);
    iterm += _KI*error_curr;
    control = pterm + dterm+ iterm;//

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control/3;

    //duty_target = min(max(duty_target, _DUTY_MIN), _DUTY_MAX); 

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; // lower limit
  if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; // upper limit
  
  // [1615] 마지막 샘플링 시각 업데이트
  last_sampling_time_dist += _INTERVAL_DIST;
  error_prev = error_curr;
  }
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval

    // update servo position
    myservo.writeMicroseconds(_DUTY_NEU*2-duty_target); // [1615]



    // [1615] 마지막 서보 조작 시각 업데이트
    last_sampling_time_servo += _INTERVAL_SERVO;
  }
  
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(ema_dist);
    Serial.print(",T:");
 Serial.print(dist_target);
 Serial.print(",P:");
 Serial.print(map(pterm,-1000,1000,510,610));
 Serial.print(",D:");
 Serial.print(map(dterm,-1000,1000,510,610));
 Serial.print(",I:");
 Serial.print(map(iterm,-1000,1000,510,610));
 Serial.print(",DTT:");
 Serial.print(map(duty_target,1000,2000,410,510));
 Serial.print(",DTC:");
 Serial.print(map(duty_curr,1000,2000,410,510));
 Serial.println(",-G:245,+G:265,G:255,m:0,M:800");

    // [1615] 마지막 Serial 업데이트 시각 업데이트
    last_sampling_time_serial += _INTERVAL_SERIAL;
  }
}
/*
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = (270000 * pow(volt, -1.165)-60);
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  //return ir_distance(); // for now, just use ir_distance() without noise filter.
  static float val = _DIST_TARGET; // [3088]
  static float dist_ema  = 0; // [3088]
  float raw = ir_distance(); // [3088]
  if (raw >= _DIST_MIN && raw <= _DIST_MAX) // [3088]
    val = raw; // [3088]
  dist_ema =  (1.0 - _DIST_ALPHA) * val + dist_ema; // [3088]
  return dist_ema; // [3088]
}

*/
