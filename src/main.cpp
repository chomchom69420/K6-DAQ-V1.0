//Master -- Collect 2 Pot, 2 Wss and 1 MPU and send on SPI

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#define HSPI_MISO   12
#define HSPI_MOSI   13
#define HSPI_SCLK   11
#define HSPI_SS     15
#define LED 26

#define PULSE_CNT 6

hw_timer_t *timer_1 = NULL;

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050

int16_t AcX, AcY, AcZ,GyX, GyY, GyZ;
int16_t accAngleX,accAngleY,gyroAngleX,gyroAngleY,gyroAngleZ;
uint8_t roll,pitch,yaw;

double AccErrorX,AccErrorY,GyroErrorX,GyroErrorY,GyroErrorZ;
double elapsedTime,currentTime,previousTime;

uint8_t pot_left_rear;
uint8_t pot_right_rear;

uint8_t count_left=0;
uint8_t count_right=0;
uint8_t rpm_rear_left = 0;
uint8_t rpm_rear_right = 0;
int64_t time_diff_rear_left=0;
int64_t time_diff_rear_right=0;
int64_t time_rear_left=0;
int64_t time_rear_right=0;

int c =0;

uint16_t data1 , data2;
static const int spiClk = 1000000; 
SPIClass * hspi = NULL;

TaskHandle_t Task1;
TaskHandle_t Task2;

void calc_IMU_error();
void Task1code(void*);
void Task2code(void*);
void test_core0();
void test_core1();
void get_mpu_data();
void send_spi(uint8_t);
void get_pot_data();
void handler();
void IRAM_ATTR wss_rear_left_isr();
void IRAM_ATTR wss_rear_right_isr();
void calc_rpm_rear_left();
void calc_rpm_rear_right();

void setup()
{
  Serial.begin(9600);
  pinMode(32,INPUT);
  pinMode(33,INPUT);
  pinMode(34,INPUT);
  pinMode(35,INPUT);
  pinMode(LED,OUTPUT);
  pinMode(23,OUTPUT);
  digitalWrite(LED,LOW);
  digitalWrite(23,LOW);
  Wire.begin(21, 22, 100000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  hspi = new SPIClass(HSPI);
  hspi->begin();
  pinMode(HSPI_SS, OUTPUT);
  xTaskCreatePinnedToCore(Task1code,"Task1",10000,NULL,1,&Task1,0);
  delay(150);
  xTaskCreatePinnedToCore(Task2code,"Task2",10000,NULL,1,&Task2,1);
  delay(150);
  calc_IMU_error();
  delay(20);

 /*
  timer_1 = timerBegin(0,80,true);
  timerAttachInterrupt(timer_1, &handler, true);
  timerAlarmWrite(timer_1, 1000000, true);
  timerAlarmEnable(timer_1);
  */

  //interrupt for WSS
  attachInterrupt(34, wss_rear_left_isr, RISING);
  attachInterrupt(35, wss_rear_right_isr, RISING);

  //timer for wss rear left
  esp_timer_early_init();

}

//Core 0 
//MPU Data collect --> Transfer

void Task1code(void *pvParameters)
{
  for(;;) {
    get_mpu_data();
    send_spi(pitch);
    send_spi(roll);
    send_spi(yaw);
    test_core0();
  }
}

//Core 1
//Pot + WSS Data collect --> Transfer

void Task2code(void *pvParameters)
{
  for(;;) {
    get_pot_data();
    // calc_rpm_rear_left();
    // calc_rpm_rear_right();
    send_spi(pot_left_rear);
    send_spi(pot_right_rear);
    //send_spi(rpm_rear_left);
    //send_spi(rpm_rear_right);
    test_core1();
  }
}

//Sending data on SPI

void send_spi(uint8_t data)
{
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(data);
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();
  delay(100);
}

void get_mpu_data()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, 1);
  AcX = (Wire.read() << 8 | Wire.read())/16384.0; 
  AcY = (Wire.read() << 8 | Wire.read())/16384.0; 
  AcZ = (Wire.read() << 8 | Wire.read())/16384.0;
  accAngleX = (atan(AcY/sqrt(pow(AcX,2)+pow(AcZ,2)))*180/PI)-AccErrorX;
  accAngleY = (atan(-1*AcX/sqrt(pow(AcY,2)+pow(AcZ,2)))*180/PI)-AccErrorY;
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime-previousTime)/1000;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,6,1);
  GyX = (Wire.read() << 8 | Wire.read())/131.0; 
  GyY = (Wire.read() << 8 | Wire.read())/131.0; 
  GyZ = (Wire.read() << 8 | Wire.read())/131.0;
  GyX = GyX -GyroErrorX;
  GyY = GyY -GyroErrorY;
  GyZ = GyZ -GyroErrorZ;
  gyroAngleX = gyroAngleX + GyX*elapsedTime;
  gyroAngleY = gyroAngleY + GyY*elapsedTime;
  yaw = yaw + GyZ*elapsedTime;
  roll = 0.96*gyroAngleX + 0.04*accAngleX;
  pitch = 0.96*gyroAngleY + 0.04*accAngleY;

}

void calc_rpm_rear_left(){
  rpm_rear_left = (count_left/PULSE_CNT)/(time_diff_rear_left/1000000)*60; 
}

void calc_rpm_rear_right(){
  rpm_rear_right = (count_right/PULSE_CNT)/(time_diff_rear_right/1000000)*60; 
}

void calc_IMU_error()
{
  while(c<200)
  {
    Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, 1);
  AcX = (Wire.read() << 8 | Wire.read())/16384.0; 
  AcY = (Wire.read() << 8 | Wire.read())/16384.0; 
  AcZ = (Wire.read() << 8 | Wire.read())/16384.0;
  AccErrorX = AccErrorX + (atan(AcY/sqrt(pow(AcX,2)+pow(AcZ,2)))*180/PI);
  AccErrorY = AccErrorY + (atan(-1*AcX/sqrt(pow(AcY,2)+pow(AcZ,2)))*180/PI);
  c++;
  }
  AccErrorX = AccErrorX/200;
  AccErrorY = AccErrorY/200;
  c=0;
  while(c<200)
  {
    Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,6,1);
  GyX = (Wire.read() << 8 | Wire.read())/131.0; 
  GyY = (Wire.read() << 8 | Wire.read())/131.0; 
  GyZ = (Wire.read() << 8 | Wire.read())/131.0;
  GyroErrorX = GyroErrorX + GyX;
  GyroErrorY = GyroErrorY + GyY;
  GyroErrorZ = GyroErrorZ + GyZ;
  }
  GyroErrorX = GyroErrorX/200;
  GyroErrorY = GyroErrorY/200;
  GyroErrorZ = GyroErrorZ/200;
}

void get_pot_data()

{
  pot_left_rear = ((analogRead(32)) << 4 | (analogRead(32)));
  pot_right_rear = ((analogRead(33)) << 4 | (analogRead(33)));
}

//WSS rear left interrupt service routine

void IRAM_ATTR wss_rear_left_isr(){
  count_left++;
  time_diff_rear_left = esp_timer_get_time() - time_rear_left;
  time_rear_left = esp_timer_get_time();
}

void IRAM_ATTR wss_rear_right_isr(){
  count_right++;
  time_diff_rear_right = esp_timer_get_time() - time_rear_right;
  time_rear_right = esp_timer_get_time();
}

void test_core0()
{
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
  delay(500);
}

void test_core1()
{
  digitalWrite(23,HIGH);
  delay(500);
  digitalWrite(23,LOW);
  delay(500);
}



void loop()
{
}