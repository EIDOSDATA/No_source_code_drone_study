void stdPID(float &setpoint, float &input, float &prev_input, float &kp, float &ki, float &kd, float &iterm, float &output)
{
    float error;
    float dInput;
    float pterm, dterm;

    error = setpoint - input;    // 오차 = 설정값 - 현재값
    dInput = input - prev_input; // 변화량 = 입력값 - 이전 입력값
    prev_input = input;          // 다움 주기에 사용하기 위해서 현재 입력값 저장

    // PID 제어
    pterm = kp * error;        // 비례항 / P제어
    iterm += ki * error * dt;  // 적분항 : 현재 오차와 센서 입력 주기(dt) 값을 곱함
    dterm = -kd * dInput / dt; // 미분항 : 외력에 의한 변경이므로 setpoint에 의한 내부적인 요소를 제외해야함. (-) 추가

    output = pterm + iterm + dterm // Output 값으로 PID 요소를 합산함.

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
}

///////////////
/* PID 기본 형태 */
void clacYPRtoStdPID()
{
    // ROLL PID
    stdPID(
        roll_target_angle,
        filtered_angle_y,
        roll_prev_angle,
        roll_kp,
        roll_ki,
        roll_kd,
        roll_iterm,
        roll_output);

    // Pitch PID
    stdPID(
        pitch_target_angle,
        filtered_angle_x,
        pitch_prev_angle,
        pitch_kp,
        pitch_ki,
        pitch_kd,
        pitch_iterm,
        pitch_output);

    // Yaw PID
    stdPID(
        yaw_target_angle,
        filtered_angle_z,
        yaw_prev_angle,
        yaw_kp,
        yaw_ki,
        yaw_kd,
        yaw_iterm,
        yaw_output);
}

void calcMotorSpeed()
{
    motorA_speed = (throttle == 0) ? 0 : throttle + yaw_output + roll_output + pitch_output; // +++
    motorB_speed = (throttle == 0) ? 0 : throttle - yaw_output - roll_output + pitch_output; // --+
    motorC_speed = (throttle == 0) ? 0 : throttle + yaw_output - roll_output - pitch_output; // +--
    motorD_speed = (throttle == 0) ? 0 : throttle - yaw_output + roll_output - pitch_output; // -+-

    // Motor A
    if (motorA_speed < 0)
        motorA_speed = 0;
    if (motorA_speed > 255)
        motorA_speed = 255;
    // Motor B
    if (motorB_speed < 0)
        motorB_speed = 0;
    if (motorB_speed > 255)
        motorB_speed = 255;
    // Motor C
    if (motorC_speed < 0)
        motorC_speed = 0;
    if (motorC_speed > 255)
        motorC_speed = 255;
    // Motor D
    if (motorD_speed < 0)
        motorD_speed = 0;
    if (motorD_speed > 255)
        motorD_speed = 255;

    /* 설명
    motorA_speed : Yaw 출력값의 방향 동일 / Roll 출력값의 방향과 동일 / Pitch 출력값의 방향과 동일
    motorB_speed : Yaw 출력값의 방향 반대 / Roll 출력값의 방향과 반대 / Pitch 출력값의 방향과 동일
    motorC_speed : Yaw 출력값의 방향 동일 / Roll 출력값의 방향과 반대 / Pitch 출력값의 방향과 반대
    motorD_speed : Yaw 출력값의 방향 반대 / Roll 출력값의 방향과 동일 / Pitch 출력값의 방향과 반대
    */
}


/* 2중 PID

표준 PID 로는 드론을 제대로 띄울 수 없었다.
드론이 기울어긴 순간에 다시 제 자리로 돌아기기 위해서는 두가지 요소가 고려되야 하기 때문이다.

첫번째 : 현재 상테에서의 오차이다. 오차에 따라 모터의 속도가 달라지게 된다.
두번째 : 현재 상테에서의 회전속도. 회전속도와 방향도 모터의 속도에 영향을 주게 된다.

// Git Hub 그림 필요함

Roll 제어를 이중 루프 PID 제어.
일차적으로 가속도 센서를 통해 얻은 각도 오차에 대해 P, I 값으 한다.
자이로 센서를 통해 얻은 각속도 오차에 대해 P, I 값을 구하고 최종적으로 더해 출력값을 계산한다.

*/



float roll_target_angle = 0.0;
float roll_angle_in;
float roll_rate_in;
float roll_stabilize_kp;
float roll_stabilize_ki;
float roll_rate_kp;
float roll_rate_ki;
float roll_stabilize_iterm;
float roll_rate_iterm;
float rolll_output;

float pitch_target_angle = 0.0;
float pitch_angle_in;
float pitch_rate_in;
float pitch_stabilize_kp;
float pitch_stabiliz_ki;
float pitch_rate_kp;
float pitch_rate_ki;
float pitch_stabilize_iterm;
float pitch_rate_iterm;
float pitch_output;

float yaw_target_angle = 0.0;
float yaw_angle_in;
float yaw_rate_in;
float yaw_stabilize_kp;
float yaw_stabilize_ki;
float yaw_rate_kp;
float yaw_rate_ki;
float yaw_stabilize_iterm;
float yaw_rate_iterm;
float yaw_output;

// 표준 PID 방식에 각도와 각속도 매개 변수가 추가되었다.
// 일반 비례항, 적분항과 같이 안정화 비례항, 적분항이 추가되었다. 미분항은 사용하지 않는다.

// 이중 PID는 ANGLE PID Controller 이후 Rate PID Controller를 거친다.
/*
인입 각도(Roll, Pitch)는 상보필터로 구한 값을 넣고, 인입 각속도(Yaw)는 자이로 센서로부터 얻은 값을 넣는다.
*/