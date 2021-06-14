# Drone PID Study
<h1 align="center">PID 알고리즘 이해하기</h1>

<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121832266-a6fb9180-cd04-11eb-8e19-e09455ad33fb.png" width="720" /></p>

<p align="center">PID 블록 다이어그램</p>

## :pencil2: 설명

- PID는 비례(Proportion), 적분(Integral), 미분(Differential) 를 통한 제어 기법이다.
- 기본적으로 피드백 구조로 되어 있다.


### :pencil2: 특징
- 제어하고자 하는 대상의 입력 값(INPUT)을 측정하여 이를 목표 설정값(SetPoint)과 비교하여 오차를 계산한다.
- 이 때 오차값을 이용하여 제어에 필요한 제어값(OUTPUT)을 계산하고, 이 제어값은 다시 피드백 되어 제어하고자 하는 대상의 입력으로 사용되는 구조다.

쉽게 날려서 설명하자면 입력 >> 목표치 도달설정 >> 오차값으로 필요 제어량 계산 >> 피드백 이다.


### :pencil2: 수식 이해하기
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121833193-c7c4e680-cd06-11eb-9dfa-8901c80c30a6.png" width="480" /></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121833277-f478fe00-cd06-11eb-9636-39aa591a04ed.png" width="320" /></p>

<b>각각의 역할은 다음과 같다.</b>

- 비례항 : 현제 상태에서의 오차 값의 크기에 비례한 제어를 한다.
- 적분항 : 일정한 상태로 유지되는 오차를 없애는 작용한 한다.
- 미분항 : 출력값의 급격한 변화에 제동을 걸어 목표값을 지나가 버리는 오버슛을 줄여 안정성을 향상시킨다.

<h3>비례, 적분, 미분 제어에 따른 변화</h3>
<h4>1. 초기상태</h4>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121833943-79184c00-cd08-11eb-9b52-c76e71d6b625.png"/></p>

<h4>2. 비례항 (P)</h4>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834379-9a2d6c80-cd09-11eb-873a-474651935e34.png"/></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834035-b7157000-cd08-11eb-887d-6c5513558dbd.png"/></p>

- 그래프에서 보면 Kp값이 5까지 증가하면서 SetPoint는 빨리 도달하였다. (피크점) 
- 하지만 전체적으로 오차가 너무 심하고 시스템이 불안정 한 것을 확일 할 수 있다.

<h4>3. 적분항 (P >> I)</h4>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834405-b204f080-cd09-11eb-9593-198c1de62f16.png"/></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834056-c72d4f80-cd08-11eb-8f63-ba9040e10676.png"/></p>

- <b>비례방식으로 드론을 제어하기엔 무리임을 확인하였다.</b> 
- <b>적분항은 시간에 걸친 오차의 누적된 합을 말한다.</b> 오차가 P항에서처럼 클 경우 적분항에 의해 시간이 지나면서 오차의 합이 쌓여 출력은 빠르게 피드백을 함으로 오차를 제거한다.
- 위 그래프를 보면 비례항 대비 오차가 줄어들었고 시스템이 나름 안정화 된 것을 볼 수 있다.

<h4>4. 미분항 (P >> I >> D)</h4>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834433-c0eba300-cd09-11eb-97b0-51f59ebbf13f.png"/></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834082-d3b1a800-cd08-11eb-9ab2-5f6a52307599.png"/></p>

- 미분항은 오차의 변화율을 계산하고 그 결과를 출력에 더한다. 
- 오차의 변화가 크지 않다면 미분항의 값은 작아지게 되고 출력에는 큰 영향을 미치지 않는다. (가끔 그래서 제어할 때 D를 생략하는 이중 PID가 있다.)
- 하지만 오차가 크게 변한다면 시스템의 진동을 피하기 위해서 미분항의 값은 커진다.
- 위 그래프는 적분항에 있던 오차를 변화율에 따라 제거함으로 이제 겨우 제어 할 만한 수준까지 안정화 된다.
 
## :tada: 표준 PID 알고리즘

- 기본 코드다.
- 지글러 니콜스 방식이 적용되지 않아 있으므로 알고리즘만 대강 알아두면 된다.
- 이런 알고리즘을 Roll, Pith, Yaw 에 각각 해당하도록 만들면 된다. 

```C
float setpoint; // 목표값
float input; // 수신받은값
float prev_input; // 이전에 수신 받은 값

float kp; // 비례 매개변수
float ki; // 적분 매개변수
float kd; // 미분 매개변수

float pterm; // 비례항
float iterm; // 적분항
float dterm; // 미분항

float output; // 출력값
float err; // 에러값
float dinput; // 변화량(오차율) 계산용

err = setpoint - input;    // 오차 = 설정값 - 현재값
dInput = input - prev_input; // 변화량 = 입력값 - 이전 입력값
prev_input = input;          // 다움 주기에 사용하기 위해서 현재 입력값 저장

// PID 제어
pterm = kp * error;        // 비례항 / P제어
iterm += ki * error * dt;  // 적분항 : 현재 오차와 센서 입력 주기(dt) 값을 곱함 >> IMU 센서 주기 
dterm = -kd * dInput / dt; // 미분항 : 외력에 의한 변경이므로 setpoint에 의한 내부적인 요소를 제외해야함. (-) 추가

output = pterm + iterm + dterm // Output 값으로 PID 요소를 합산함.
// >> motor1 = output(roll, pitch, yaw) sum.


/* 시퀀스 설명
1. 오차를 구함
2. 변화를 측정하고 다움 주기에 사용하기 위한 값 저장
3. PID 구함
    3.1 P항 >> 비례매개변수 * 현재 오차
    3.2 I항은 시간에 건친 오차의 합으로 >> I항 = I항 + (적분 매개변수 * 현재 오차 * 센서주기)
    3.3 D항은 오차의 변화율 계산이므로 >> 미분 매개변수 * 변화량 / 센서주기
        >> dterm = -kd * (input-prev_input) / dt
4. 미분항 부호가 음수인 이유
    미분항은 갑작스러운 외력의 변화에 저항하기 위해 사용된다.
    SetPoint의 변화 즉 내부적인 의도에 의한 변하에는 논리적으로 적용이 불가하다.
    SetPoint가 변하는 상황을 미분항에서 빼면 부호가 바뀌게 된다.        
*/
```


- Roll, Pitch, Yaw 적용

```C
// MOTOR PARAMETER
float motor_1_speed, motor_2_speed, motor_3_speed, motor_4_speed;
volatile long int cnt_rising = 0;
volatile long int cnt_falling = 0;

volatile long int cnt_elev = 0;     
volatile long int cnt_aile = 0;
volatile long int cnt_thro = 0;
volatile long int cnt_rudd = 0;
volatile long int cnt_aux2 = 0;

// ROLL Variable
float r_tgt_angle; // 목표값
float r_input_y; // 수신받은값
float r_prev_input_y; // 이전에 수신 받은 값

float r_pterm; // 비례항
float r_iterm; // 적분항
float r_dterm; // 미분항

float r_p; // 비례 매개변수 >> GAIN
float r_i; // 적분 매개변수 >> GAIN
float r_d; // 미분 매개변수 >> GAIN

float r_err; // 에러값
float r_d_input; // 변화량(오차율) 계산용
float r_output; // 모터에 대한 출력값

// PITCH Variable
float p_tgt_angle; // 목표값
float p_input_x; // 수신받은값
float p_prev_input_x; // 이전에 수신 받은 값

float p_pterm; // 비례항
float p_iterm; // 적분항
float p_dterm; // 미분항

float p_p; // 비례 매개변수 >> GAIN
float p_i; // 적분 매개변수 >> GAIN
float p_d; // 미분 매개변수 >> GAIN

float p_err; // 에러값
float p_d_input; // 변화량(오차율) 계산용
float p_output; // 모터에 대한 출력값

// YAW Variable
float y_tgt_angle; // 목표값
float y_input_z; // 수신받은값
float y_prev_input_z; // 이전에 수신 받은 값

float y_pterm; // 비례항
float y_iterm; // 적분항
float y_dterm; // 미분항

float y_p; // 비례 매개변수 >> GAIN
float y_i; // 적분 매개변수 >> GAIN
float y_d; // 미분 매개변수 >> GAIN

float y_err; // 에러값
float y_d_input; // 변화량(오차율) 계산용
float y_output; // 모터에 대한 출력값

// ROLL PID 제어
r_err = r_tgt_angle - r_input;    // 오차 = 설정값 - 현재값
r_d_input = r_input_y - r_prev_input_y; // 변화량 = 입력값 - 이전 입력값
r_prev_input_y = r_input_y;          // 다음 주기에 사용하기 위해서 현재 입력값 저장

r_pterm = r_p * r_error;        // 비례항 / P제어
r_iterm += r_i * r_err * dt;  // 적분항 : 현재 오차와 센서 입력 주기(dt) 값을 곱함 >> IMU 센서 주기 
r_dterm = -r_d * r_d_input / dt; // 미분항 : 외력에 의한 변경이므로 setpoint에 의한 내부적인 요소를 제외해야함. (-) 추가

r_output = r_pterm + r_iterm + r_dterm // Output 값으로 PID 요소를 합산함.

// PITCH PID 제어
p_err = p_tgt_angle_x - p_input_x;    // 오차 = 설정값 - 현재값
p_d_input = p_input_x - p_prev_input_x; // 변화량 = 입력값 - 이전 입력값
p_prev_input_x = p_input_x;          // 다음 주기에 사용하기 위해서 현재 입력값 저장

p_pterm = p_p * p_err;        // 비례항 / P제어
p_iterm += p_i * p_err * dt;  // 적분항 : 현재 오차와 센서 입력 주기(dt) 값을 곱함 >> IMU 센서 주기 
p_dterm = -p_d * p_d_input / dt; // 미분항 : 외력에 의한 변경이므로 setpoint에 의한 내부적인 요소를 제외해야함. (-) 추가

p_output = y_pterm + y_iterm + y_dterm // Output 값으로 PID 요소를 합산함.

// YAW PID 제어
y_err = y_tgt_angle_z - y_input_z;    // 오차 = 설정값 - 현재값
y_d_input = y_input_z - y_prev_input_z; // 변화량 = 입력값 - 이전 입력값
y_prev_input_z = y_input_z;          // 다음 주기에 사용하기 위해서 현재 입력값 저장

y_pterm = y_p * y_err;        // 비례항 / P제어
y_iterm += y_i * y_err * dt;  // 적분항 : 현재 오차와 센서 입력 주기(dt) 값을 곱함 >> IMU 센서 주기 
y_dterm = -y_d * y_d_input / dt; // 미분항 : 외력에 의한 변경이므로 setpoint에 의한 내부적인 요소를 제외해야함. (-) 추가

y_output = y_pterm + y_iterm + y_dterm // Output 값으로 PID 요소를 합산함.

// "+" 자형 쿼드콥터
//>> +++, --+, +--, -+-
motor_1_speed = (throttle == 0) ? 0 : throttle + y_output + r_output + p_output;
motor_2_speed = (throttle == 0) ? 0 : throttle - y_output - r_output + p_output;
motor_3_speed = (throttle == 0) ? 0 : throttle + y_output - r_output - p_output;
motor_4_speed = (throttle == 0) ? 0 : throttle - y_output + r_output - p_output;

// "x" 자형 쿼드콥터
//>> -++, +-+, ---, ++-
motor_1_speed = (throttle == 0) ? 0 : throttle - y_output + r_output + p_output;
motor_2_speed = (throttle == 0) ? 0 : throttle + y_output - r_output + p_output;
motor_3_speed = (throttle == 0) ? 0 : throttle - y_output - r_output - p_output;
motor_4_speed = (throttle == 0) ? 0 : throttle + y_output + r_output - p_output;

// motor_n_speed 로 PWM 제어를 한다.
// throttle 신호는 PWM으로 받아온다.

```



## :mag: 라이센스

![cc license](http://i.creativecommons.org/l/by/4.0/88x31.png)

이 가이드는 Creative Commons Attribution 4.0 (CCL 4.0)을 따릅니다.

