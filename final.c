#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <pthread.h> //쓰레드 관련 함수
#include <wiringSerial.h>

#define PWM1 19 // 조도센서 관련 LED
#define PWM2 24 // 경계모드 관련 버튼
#define PWM3 12 // 경계모드 관련 LED

#define HIGH 1
#define LOW 0

#define TRIG 15
#define ECHO 18

#define BAUD_RATE 115200

#define SLAVE_ADDR_01 0x48
static const char *I2C_DEV = "/dev/i2c-1";     // I2C
static const char *UART2_DEV = "/dev/ttyAMA1"; // UART2 연결을 위한 장치 파일

// 경계모드 ON/OFF (1 일때 ON, 0 일때 OFF)
int state = 0;
// 뮤텍스 오브젝트
pthread_mutex_t mid;

// 쓰레드에서 호출할 함수
// 조도 센서를 감지하여 LED on/off 기능
void *threadFunc1(void *data)
{
    int i2c_fd;             // i2c
    int val = 0;            // DAC
    float outVoltage = 0.0; // DAC
    int preVal = 0;         //
    int curVal = 0;         //
    int threshold = 200;    //
    int adcChannel = 0;
    int i;

    // wiringPi
    if (wiringPiSetupGpio() < 0)
    {
        printf("wiringPiSetup() is failed\n");
    }
    // i2C
    if ((i2c_fd = wiringPiI2CSetupInterface(I2C_DEV, SLAVE_ADDR_01)) < 0)
    {
        printf("wiringPi2CSetup Failed: \n");
    }
    while (1)
    {
        wiringPiI2CWrite(i2c_fd, 0x40 | adcChannel);
        preVal = wiringPiI2CRead(i2c_fd);        // (0~255)
        curVal = wiringPiI2CRead(i2c_fd);        // ADC
        printf("Previous value = %d, ", preVal); //
        printf("Current value= %d, ", curVal);   //
        // threshold , Bright
        if (curVal < threshold)
        {
            printf("Bright!\n");
            i++;
        }
        else
            printf("Dark!\n");

        wiringPiI2CWriteReg8(i2c_fd, 0x40, curVal); // Analog output enable: 1 (active)
        outVoltage = ((float)curVal / 255) * 3.3;   //
        printf("Raw value : %d, Output Voltage : %.3f [V]\n", curVal, outVoltage);
        if (i == 2)
            return (void *)i;
        sleep(1);
    }
}

// 쓰레드에서 호출할 함수
// 버튼 누르면 경계모드 on/off 기능
void *threadFunc2(void *data)
{
    int alert = 0;
    int value = 1;
    int i = 0;

    pinMode(PWM2, INPUT);  // 버튼
    pinMode(PWM3, OUTPUT); // LED

    while (1)
    {
        // wiringPi
        if (wiringPiSetupGpio() < 0)
        {
            printf("wiringPiSetup() is failed\n");
        }

        pthread_mutex_lock(&mid);

        // 임계 구역
        // PWM2 버튼을 눌렀을 시 HIGH, LOW 값 반환
        state = digitalRead(PWM2);

        if ((state == HIGH) && (alert == LOW))
        {
            value = 1 - value;
        }
        alert = state;

        pthread_mutex_unlock(&mid);

        // 경계모드 on
        if (value == HIGH)
        {
            digitalWrite(PWM3, HIGH);
            i++;
        }
        else
        {
            digitalWrite(PWM3, LOW);
        }

        if (i == 1)
            return (void *)alert;
    }
    sleep(1);
}

int wait_state(int state)
{
    while (digitalRead(ECHO) == state)
        ;
    return 0;
}

// 여러 바이트의 데이터를 씀
void serialWriteBytes(const int fd, const char *s)
{
    write(fd, s, strlen(s));
}

// 1Byte 데이터를 수신하는 함수
unsigned char serialRead(const int fd)
{
    unsigned char x;
    if (read(fd, &x, 1) != 1) // read 함수를 통해 1바이트 읽어옴
        return -1;
    return x; // 읽어온 데이터 반환
}

// 1Byte 데이터를 송신하는 함수
void serialWrite(const int fd, const unsigned char c)
{
    write(fd, &c, 1); // write 함수를 통해 1바이트 씀
}

// 쓰레드에서 호출할 함수
// 초음파 센서 감지하여 스마트폰 알림 전송
void *threadFunc3(void *data)
{
    int fd_serial; // UART2 파일 서술자
    long start_time, end_time;
    float distance;

    if (wiringPiSetupGpio() == -1)
    {
        exit(1);
    }

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    digitalWrite(TRIG, LOW);

    while (1)
    {
        if (state == HIGH)
        {
            digitalWrite(TRIG, HIGH);
            delayMicroseconds(10);
            digitalWrite(TRIG, LOW);

            wait_state(LOW);
            start_time = micros();
            wait_state(HIGH);
            end_time = micros();
            distance = (end_time - start_time) / (58.) * (10.);

            if ((fd_serial = serialOpen(UART2_DEV, BAUD_RATE)) < 0)
            { // UART2 포트 오픈
                printf("Unable to open serial device.\n");
            }
            char dat[100];
            if (distance < 1500)
            {
                printf("distance : %lf이하에 접근하였습니다.\n", distance);
                sprintf(dat, "%lfmm 이내에 접근하였습니다. \n", distance);
                serialWriteBytes(fd_serial, dat); // 입력 받은 데이터를 다시 보냄 (Echo)
            }
        }
        sleep(1);
    }
}

int main(void)
{

    pthread_t p_thread1; // thread ID 저장 변수
    pthread_t p_thread2; // thread ID 저장 변수
    pthread_t p_thread3; // thread ID 저장 변수

    pthread_mutex_init(&mid, NULL); // 뮤텍스 초기화

    // thread 1 (조도 센서 스레드) 생성
    if (pthread_create(&p_thread1, NULL, threadFunc1, NULL) < 0)
    { // 반환 값이 0보다 작으면 쓰레드 생성 오류
        perror("pthread_create() error\n");
        exit(0);
    }

    // thread 1가 종료될 때까지 대기
    if (pthread_join(p_thread1, NULL) < 0)
    { // 반환 값이 0보다 작으면 pthread_join 오류
        perror("pthread_join() error\n");
        exit(0);
    }

    // thread 2 : (경계 모드 버튼 ON/OFF 및 LED 스레드) 생성
    if (pthread_create(&p_thread2, NULL, threadFunc2, NULL) < 0)
    { // 반환 값이 0보다 작으면 쓰레드 생성 오류
        perror("pthread_create() error\n");
        exit(0);
    }

    // thread 2가 종료될 때까지 대기
    if (pthread_join(p_thread2, NULL) < 0)
    { // 반환 값이 0보다 작으면 pthread_join 오류
        perror("pthread_join() error\n");
        exit(0);
    }

    // thread 3 (초음파 센서 스레드) 생성
    if (pthread_create(&p_thread3, NULL, threadFunc3, NULL) < 0)
    { // 반환 값이 0보다 작으면 쓰레드 생성 오류
        perror("pthread_create() error\n");
        exit(0);
    }

    // thread 3가 종료될 때까지 대기
    if (pthread_join(p_thread3, NULL) < 0)
    { // 반환 값이 0보다 작으면 pthread_join 오류
        perror("pthread_join() error\n");
        exit(0);
    }
    pthread_mutex_destroy(&mid); // 뮤텍스 제거
    return 0;
}