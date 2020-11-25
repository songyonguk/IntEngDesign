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

// Distance sensor
#define _DIST_ALPHA 0.0 // [3095] ema필터의 alpha 값을 설정

// Servo range
#define _DUTY_MIN 800   // [1615] 서보 제어 펄스 폭: 최고 각도
#define _DUTY_NEU 1550  // [1615] 서보 제어 펄스 폭: 수평
#define _DUTY_MAX 2200  // [1615] 서보 제어 펄스 폭: 최저 각도

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 30 

// Event periods
#define _INTERVAL_DIST 20  // [3074] 거리측정주기 (ms)
#define _INTERVAL_SERVO 20 // [3078] 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // [3078] Serial제어주기 (ms)

#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.35     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.

// PID parameters
#define _KP 0.8 //[0711] 비례이득

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;
const float coE[] = {0.0000840, -0.0206750, 2.5061554, 2.3103917};
// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;  // [1615] 센서가 인식한 거리/ema 필터링된 거리

float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3; 

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED, OUTPUT); // [3083] PIN_LED 핀을 OUTPUT으로 설정
myservo.attach(PIN_SERVO); // [3083] PIN_SERVO 핀을 servo를 쓰는 핀으로 설정

// initialize global variables

// move servo to neutral position
myservo.writeMicroseconds(1550); // [3090]

// initialize serial port
Serial.begin(57600); // [3083] serial port의 baud rate를 57600으로 초기화
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
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
float ir_distance(void){ // return value unit: mm
  // [3088] 19_example_0에서 가져옴
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;  // [1615] 변환 식 보정 필요
  return val;
}

//float ir_distance_filtered(void){ // return value unit: mm
  //return ir_distance(); // for now, just use ir_distance() without noise filter.
 // static float val = _DIST_TARGET; // [3088]
 // static float dist_ema  = 0; // [3088]
 // float raw = ir_distance(); // [3088]
 // if (raw >= _DIST_MIN && raw <= _DIST_MAX) // [3088]
 //   val = raw; // [3088]
 // dist_ema =  (1.0 - _DIST_ALPHA) * val + dist_ema; // [3088]
 // return dist_ema; // [3088]
//}
  
void loop() {

/////////////////////
// Event generator //
/////////////////////
// [1615] 거리 측정 주기가 되었는지 검사 
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
  // get a distance reading from the distance sensor
      dist_raw = ir_distance();

  // PID control logic
    error_curr = (dist_raw - _DIST_TARGET); // [3073] 현재 읽어들인 데이터와 기준 값의 차이
    pterm = (_KP*error_curr);// [3073] p게인 값인 kp와 error 값의 곱
    control = //

  // duty_target = f(duty_neutral, control)
   duty_target = 

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
 duty_target = min(max(duty_target, _DUTY_MIN), _DUTY_MAX); // [1615]
  
  // [1615] 마지막 샘플링 시각 업데이트
  last_sampling_time_dist += _INTERVAL_DIST;
  }
  
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval

    // update servo position
    myservo.writeMicroseconds(1250-(float)(pterm*4)); // [1615]
//    myservo.writeMicroseconds((float)1550+(pterm*4)); // [1615]
    // [1615] 마지막 서보 조작 시각 업데이트
    last_sampling_time_servo += _INTERVAL_SERVO;
  }
unsigned long time_curr = millis();
    // calibrate distance reading from the IR sensor
  float x = ir_distance();
 dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  if(event_dist) {
    event_dist = false;
    dist_raw = ir_distance();}
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

    // [1615] 마지막 Serial 업데이트 시각 업데이트
    last_sampling_time_serial += _INTERVAL_SERIAL;

  //  float x = ir_distance();
 //   dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  }
}
