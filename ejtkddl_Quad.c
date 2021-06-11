#include <mega128.h>
#include <stdio.h>
#include <stdlib.h>
#include <delay.h>
#include <string.h>

#define BUFFERLENGTH 100

#define SamplingTime 0.01

volatile long int cnt_rising=0;
volatile long int cnt_falling=0;
volatile long int cnt_elev=0;    // 메모리 변수 선언 (인터럽트에서 쓰이는 변수들)
volatile long int cnt_aile=0;
volatile long int cnt_thro=0;
volatile long int cnt_rudd=0;
volatile long int cnt_aux2=0;

volatile unsigned char ch;
volatile unsigned char Buf[BUFFERLENGTH];
volatile unsigned int N_RXDATA;

void PORT_init()
{
    DDRD = 0x00;     // INT0,1 수신방향으로 설정
    PORTD = 0b00000011;    // INT0,1 내부 풀업저항 사용
    
    DDRE = 0b00000010;     // INT 4,5 수신방향으로 설정
    PORTE = 0b11100000;   // INT4,5 내부 풀업저항 사용
    
    EIMSK = 0b11100011;    // INT 0,1,4,5 외부인터럽트 개별 허용
    EICRA = 0b00001111;    // INT 0,1 - rising edge 트리거 설정
    EICRB = 0b11111100;    // INT 5,6,7 - rising edge 트리거 설정
}

interrupt [EXT_INT0] void ext_int0_isr(void)   // ELEV 채널 수신
{
    if(EICRA == 0b00001111){
        cnt_rising = TCNT1;
        EICRA = 0b00001110;
    }
    else{
        cnt_falling = TCNT1;
        cnt_elev = (35556 - cnt_rising + cnt_falling) %35556;
        EICRA = 0b00001111;
    }
}

interrupt [EXT_INT1] void ext_int1_isr(void)   // AILE 채널 수신
{
    if(EICRA == 0b00001111){
        cnt_rising = TCNT1;
        EICRA = 0b00001011;
    }
    else{
        cnt_falling = TCNT1;
        cnt_aile = (35556 - cnt_rising + cnt_falling) %35556;
        EICRA = 0b00001111;
    }
}

interrupt [EXT_INT6] void ext_int6_isr(void)   // THRO 채널 수신
{
    if(EICRB == 0b11111100){
        cnt_rising = TCNT1;
        EICRB = 0b11101100;
    }
    else{
        cnt_falling = TCNT1;
        cnt_thro = (35556 - cnt_rising + cnt_falling) %35556;
        EICRB = 0b11111100;
    }
}

interrupt [EXT_INT7] void ext_int7_isr(void)   // RUDD 채널 수신
{
    if(EICRB == 0b11111100){
        cnt_rising = TCNT1;
        EICRB = 0b10111100;
    }
    else{
        cnt_falling = TCNT1;
        cnt_rudd = (35556 - cnt_rising + cnt_falling) %35556;
        EICRB = 0b11111100;
    }
}

interrupt [EXT_INT5] void ext_int5_isr(void)   // AUX2 채널 수신
{
    if(EICRB == 0b11111100){
        cnt_rising = TCNT1;
        EICRB = 0b11111000;
    }
    else{
        cnt_falling = TCNT1;
        cnt_aux2 = (35556 - cnt_rising + cnt_falling) %35556;
        EICRB = 0b11111100;
    }        
}

void UART0_init()      // 하이퍼터미널용
{
    delay_ms(10);
    DDRE |= 0b00000010;
    UCSR0A = 0x02;
    UCSR0B = 0x18;
    UCSR0C = 0x06;
    UBRR0H = 0;
    UBRR0L = 16;
}
void UART0_putch(unsigned char ch)
    {while((UCSR0A & 0x20) == 0); UDR0 = ch;} 

void UART1_init(void)   // 센서 통신용
{
    delay_ms(10);
    DDRD |= 0b00001000; 
    UCSR1A = 0x02;
    UCSR1B = 0x98;
    UCSR1C = 0x06;
    UBRR1H = 0;
    UBRR1L = 16;   
}
void UART1_putch(unsigned char ch)
    {while((UCSR1A & 0x20) == 0); UDR1 = ch;}                                                 
void UART1_putstrf(flash unsigned char *str)
    {unsigned int i=0; for(;str[i] != 0; i++) UART1_putch(str[i]);}
interrupt [USART1_RXC] void UART1_getch(void)
    {ch=UDR1; Buf[N_RXDATA]=ch; N_RXDATA=N_RXDATA+1;}
      
void Timer1_init()
{
    DDRB |= 0b11111111;        
    TCCR1A = 0b10100010;   //Fast PWM 14 Mode, 1 scaler, TOP 35555, 450Hz
    TCCR1B = 0b00011001;
    ICR1 = 35555;
}
void Timer3_init()
{
    DDRE |= 0b00011000;
    TCCR3A = 0b10100010;   //Fast PWM 14 Mode, 1 scaler, TOP 35555, 450Hz
    TCCR3B = 0b00011001;
    ICR3H = 35555 >> 8;
    ICR3L = 35555 & 0xFF;
}

void Limit_cut(float *ff, long int MIN_LIMIT, long int MAX_LIMIT)
{
    if(*ff < MIN_LIMIT)
    {
        *ff = MIN_LIMIT;
    }
    else if(*ff > MAX_LIMIT)
    {
        *ff = MAX_LIMIT;
    }
} 

void main(void)
{   
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int k = 0;
    
    char data[10];
    int VALID_DATA[14];

    float f1 = 0;
    float f2 = 0;
    float f3 = 0;
    float f4 = 0;
    
    //Roll control
    long int Roll_cmd             = 0;
    long int phi_feedback         = 0;
    long int phi_dot_feedback     = 0;
    float roll_cont               = 0;
    float ROLL_ANG_ERR            = 0;
    float ROLL_RATE_ERR           = 0;
    float ROLL_RATE_P             = 0;
    float ROLL_RATE_I             = 0;
    float ROLL_RATE_D             = 0;
    float ROLL_RATE_ERR_PRE       = 0;

    float R_inner_Pgain           = 0.10; // 2015.02.20
    float R_inner_Igain           = 0.35;
	float R_inner_Dgain           = 0.0035;  
    float R_outer_Pgain           = 4.0;

    
    //Pitch control
    long int Pitch_cmd            = 0;
    long int theta_feedback       = 0;
    long int theta_dot_feedback   = 0;
    float pitch_cont              = 0;
    float PITCH_ANG_ERR           = 0;
    float PITCH_RATE_ERR          = 0; 
    float PITCH_RATE_P            = 0;
    float PITCH_RATE_I            = 0;
    float PITCH_RATE_D            = 0;
    float PITCH_RATE_ERR_PRE      = 0;
    
    float P_inner_Pgain           = 0.10;
    float P_inner_Igain           = 0.35;
    float P_inner_Dgain           = 0.0035;
	float P_outer_Pgain           = 4.0;
    
    //Yaw Control
    long int Yaw_cmd              = 0;
    long int psi_feedback         = 0;
    long int psi_dot_feedback     = 0;
    float yaw_cont                = 0;
    float YAW_RATE_ERR            = 0;
    
    float YAW_RATE_P              = 0;
    float YAW_RATE_I              = 0;
    
    float Y_inner_Pgain           = 0.25;
    float Y_inner_Igain           = 0.55;
    
    //Thrust_cmd
    long int Throttle             = 0;
        
    PORT_init();
    UART0_init();                           
    
    UART1_init();
    Timer1_init();
    Timer3_init();             
    
    SREG |= 0x80;
    
    memset(data, 0x00, sizeof(data)); 
    memset(VALID_DATA,0x00,sizeof(VALID_DATA));
    
    while(cnt_thro > 16010 || cnt_thro < 15950 ) {}   // Starter
    OCR1A = 16000;   // 1ms Pulse Width
    OCR1B = 16000;
    OCR3BH = 16000 >> 8;
    OCR3BL = 16000 & 0xFF;
    OCR3AH = 16000 >> 8;
    OCR3AL = 16000 & 0xFF; 

    delay_ms(3000);
    
    while(1)
    {        
        // Initialization in regard to Sensor Parsing Section
        memset(Buf,0x00,sizeof(Buf));
        ch=0x00;
        N_RXDATA=0;
        
        // Control Input
        Roll_cmd  = -0.3*(cnt_aile-24000);// max 2400  (means 24 deg)
        Pitch_cmd = 0.3*(cnt_elev-24000); // max 2400  (means 24 deg)
        Yaw_cmd = 4.5*(cnt_rudd-24000);   // max 36000 (means 360 deg/s)
        Throttle = 0.8*(cnt_thro-15900);  // Set to hover at 50% Throttle
        
        //printf("aux2:%d \r\n", cnt_aux2);
        
        //R_outer_Pgain = (float)(cnt_aux2 - 17590) * 0.000078125 * 10.0 + 3.0;
        //P_outer_Pgain = R_outer_Pgain;

		// 센서 수신 및 파싱부
        UART1_putstrf("<D\r\n");             
        while(ch != 0x0a) {}
        if(Buf[0] == 0x24)
        {
            if(Buf[1] == 0x32 && Buf[2] == 0x35)      
            {
                for(i = 4; i<BUFFERLENGTH; i++)
                {
                    if(Buf[i] != 0x2c)
                    {
                        if(Buf[i] != 0x0a)
                        {
                            if(Buf[i] == 0x2E) {continue;}  // data "." skip
                            data[j] = Buf[i];
                            j=j+1;
                        }
                        else
                        {
                            break;
                        }
                    }
                    else
                    {
                        VALID_DATA[k] = atoi(data);
                        k = k+1;
                        memset(data, 0x00, sizeof(data));
                        j=0;
                    }
                }
                k=0;
            }
        }
		// FEEDBACK
        phi_feedback = VALID_DATA[1];
        phi_dot_feedback = VALID_DATA[7];
        theta_feedback = - VALID_DATA[2];
        theta_dot_feedback = - VALID_DATA[8];
        psi_feedback = - VALID_DATA[3];
        psi_dot_feedback = - VALID_DATA[9];
        
       // ROLL ANGLE Control ( ANG_P * RATE_PID controller )
        ROLL_ANG_ERR = (float)(Roll_cmd) - (float)(phi_feedback);
        ROLL_RATE_ERR = ROLL_ANG_ERR * R_outer_Pgain - phi_dot_feedback;
        ROLL_RATE_P = ROLL_RATE_ERR * R_inner_Pgain;
        ROLL_RATE_I = ROLL_RATE_I + (ROLL_RATE_ERR * R_inner_Igain) * SamplingTime;
        Limit_cut(&ROLL_RATE_I,-500,500);
        ROLL_RATE_D = (ROLL_RATE_ERR - ROLL_RATE_ERR_PRE)/SamplingTime * R_inner_Dgain;
        ROLL_RATE_ERR_PRE = ROLL_RATE_ERR;
        
        roll_cont = ROLL_RATE_P + ROLL_RATE_I + ROLL_RATE_D;
        
        // PITCH ANGLE Control ( ANG_P * RATE_PID controller )
        PITCH_ANG_ERR = (float)(Pitch_cmd) - (float)(theta_feedback);
        PITCH_RATE_ERR = PITCH_ANG_ERR * P_outer_Pgain - theta_dot_feedback;
        PITCH_RATE_P = PITCH_RATE_ERR * P_inner_Pgain;
        PITCH_RATE_I = PITCH_RATE_I + (PITCH_RATE_ERR * P_inner_Igain) * SamplingTime;
        Limit_cut(&PITCH_RATE_I,-500,500);
        PITCH_RATE_D = (PITCH_RATE_ERR - PITCH_RATE_ERR_PRE)/SamplingTime * P_inner_Dgain;
        PITCH_RATE_ERR_PRE = PITCH_RATE_ERR;
        
        pitch_cont = PITCH_RATE_P + PITCH_RATE_I + PITCH_RATE_D;
        
        // YAW RATE Control ( RATE_PI controller )                            
		YAW_RATE_ERR = Yaw_cmd - psi_dot_feedback;
        YAW_RATE_P = YAW_RATE_ERR * Y_inner_Pgain;
        YAW_RATE_I = YAW_RATE_I + YAW_RATE_ERR*Y_inner_Igain*SamplingTime;
        Limit_cut(&YAW_RATE_I,-500,500);
		yaw_cont = YAW_RATE_P + YAW_RATE_I;
        
		//모터제어값으로 믹싱
        f1 = 0.5*pitch_cont + 0.5*roll_cont - yaw_cont + (float)(Throttle);
	    f2 = 0.5*pitch_cont - 0.5*roll_cont + yaw_cont + (float)(Throttle);
	    f3 = - 0.5*pitch_cont - 0.5*roll_cont - yaw_cont + (float)(Throttle);
	    f4 = - 0.5*pitch_cont + 0.5*roll_cont + yaw_cont + (float)(Throttle);
        
        Limit_cut(&f1,0,16000);
        Limit_cut(&f2,0,16000);
        Limit_cut(&f3,0,16000);
        Limit_cut(&f4,0,16000);

		// THROTTLE 0 근처로 낮추면 시동 끔.
        if(cnt_thro < 16050 && cnt_thro > 15900 )
        {
            f1 = 0;
            f2 = 0;                       
            f3 = 0;
            f4 = 0;
                   
            ROLL_RATE_I = 0;
            PITCH_RATE_I = 0;
            YAW_RATE_I = 0;
        }
        
        // transform f value to PWM duty ratio 
        OCR1A = 16000 + (int)(f1);
        OCR1B = 16000 + (int)(f2);
        OCR3BH = 16000 + (int)(f3) >> 8;
        OCR3BL = 16000 + (int)(f3) & 0xFF;
        OCR3AH = 16000 + (int)(f4) >> 8;
        OCR3AL = 16000 + (int)(f4) & 0xFF;
    }
}
