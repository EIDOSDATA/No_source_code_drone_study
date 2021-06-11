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