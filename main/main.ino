//Проект CanSat юниор v1.1
// радио барометр-акселерометр флеш аккум фоторезистор диоды кнопки серва


/* ПОРТЫ -
    D2-D9 - светодиоды от младшего к старшему
    SD: MOSI - D11
        MISO - D12
        CS - D10
        SCK - D13
    HW612 - ТермоБароАксиГироМагни:
        SCL - A5
        SDA - A4
    SV610 - радио:
        Txd - RX0
        Rxd - TX1
    Питание радиомодуля - А0
    Кнопка 1 панели управления - А1
    Кнопка 2 панели управления - А2
    Серва - А3
    Фоторезистор - А6
    Аккум - А7

*/

/*питание радиомодуля отключается в режиме тестирования нажатием верхней кнопки
   (кнопки №2)
   включить питание на радиомодуль можно передернув питание или также как выключали-
   из тестового режима нажатием на кнопку №2.
   за включением тестового режима и питанием радиомодуля следить по панели индикации
*/

/* Ошибка индикатора №4 "Исправность" отвечает за исправность microSD и HW612
    при разряженном аккуме (последнее деление и меньше) появится эта ошибка,
    из-за сбоев в инициализации протоколов обмена по интерфейсам.
    При перезапуске возможно будет работать, но от HW612 точных показаний лучше не ждать.
    Заряжать аккумулятор перед работой и не допускать разряжения менее 50%!!!
*/

#define TeamID "Sporadic-A" // код команды
#define START_H 10 //m высота срабатывания "старта"
#define SEPARATE_A 850 // калибровочная переменная срабатывания фоторезистора
#define TIMESERVO 5000 //mc время выкручивания штивта системы спасения
#define AltRec 120 //m высота срабатывания системы спасения при движении вниз (регламент 2021 срабатывает ниже 50 метров)

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include <TimerOne.h>

#include <SD.h>
#include <SPI.h>

#define PIN_CHIP_SELECT_SD 10

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18 /*максимальный диапазон измерения. 16G: от 0 до 32767*/

Adafruit_BMP280 bme; // I2C

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Initial time to i2c
long int ti;
volatile bool intFlag = false;

bool WELL = true;

void setup() {
  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  pinMode(PIN_CHIP_SELECT_SD, OUTPUT);
  if (!SD.begin(PIN_CHIP_SELECT_SD)) {
    Serial.println("Card failed, or not present");
    WELL = false;
    //while (1);//return;
  }
  //Serial.begin(9600);
  Serial.println(F("BMP280 test"));
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    WELL = false;
    //while (1);
  }
  // Arduino initializations
  Wire.begin();
  //Serial.begin(9600);
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_2_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);
  //pinMode(13, OUTPUT); ?chip select?
  Timer1.initialize(1000); // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback); // attaches callback() as a timer overflow interrupt
  // Store initial time
  ti = millis();
}

// Counter
long int cpt = 0;

void callback() {
  intFlag = true;
  //digitalWrite(13, digitalRead(13) ^ 1);?chip select?
}

unsigned int Time = -1;
float Altitude = 0;
double A = 0;
bool Start = false,
     Separate = false,
     Recovery = false,
     Landing = false;

int SeparateF = -1,
    RecoveryN = 0,
    TimeRecToLand = 0;

float LandingH = -1;

float AbsolutH = -999;

float Temperature = -1,
      Pressure = -1;

int Battery = -1; // Заряд аккумулятора в %

bool Key1 = false,
     Key2 = false,
     TempBool = false,
     TESTING = false,
     VCCSV610 = true;
/*стартуем в боевом режиме. если в полете контроллер перезапустится, то будет в боевом режиме*/

File dataFile;

int Alt_temp_help = -111;
bool Alt_temp_help_bool = false;

void loop() {
  dataFile = SD.open("DATA.txt", FILE_WRITE);//открываем файл на microSD для записи
  Battery = analogRead(A7); /* 820 = 100%; 705 = 1%//04.04.2020 TODO сместить 0 вверх ~30%
  820=100%; 735=1%; 100/85=1.1765*/
  //Battery=(Battery-705)*0.8696;
  Battery = (Battery - 735) * 1.1765;
  if (Battery > 100) {
    Battery = 99;
  }
  if (Battery <= 0) {
    Battery = 1;
  }
  /* стартовые две секунды отображения заряда аккумулятора */
  if (Time == -1) {
    for (int i = 9, Bat = 0; i > 1, Bat < Battery; --i) {
      digitalWrite(i, HIGH);
      Bat += 13;
    }
    delay(2000);
  }
  Time = millis();
  /*готовим высоту относительно уровня старта контроллера*/
  Altitude = bme.readAltitude(1013.25);
  if (AbsolutH == -999) {
    AbsolutH = Altitude;
  }
  Altitude = Altitude - AbsolutH;

  Key1 = digitalRead(A2);
  /* Если нажали нижнюю кнопку запускается или отключается режим тестирования(в соотвтетсвии с регламентом)*/
  if (!Key1) {
    delay(10);//устраняем дребезг
    Key1 = digitalRead(A2);
    if (!Key1) {
      TESTING = !TESTING;
    }
  }
  Key2 = digitalRead(A1);
  /* Если нажали верхнюю кнопку в режиме тестирования, то отключаем/включаем радиомодуль*/
  if (!Key2 & TESTING) {
    delay(10);//устраняем дребезг
    Key2 = digitalRead(A1);
    if (!Key2) {
      VCCSV610 = !VCCSV610;
    }
  }
  if (VCCSV610) {
    digitalWrite(A0, HIGH);
  }
  else {
    digitalWrite(A0, LOW);
  }
  Temperature = bme.readTemperature();
  Temperature = Temperature - 7; // калибровка!!!
  Pressure = bme.readPressure();

  while (!intFlag);
  intFlag = false;
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
  // Create 16 bits values from 8 bits data
  // Accelerometer
  int16_t ax = -(Buf[0] << 8 | Buf[1]);
  int16_t ay = -(Buf[2] << 8 | Buf[3]);
  int16_t az = (Buf[4] << 8 | Buf[5]);
  float ax_temp = fabs(ax * 16.0 / 32767.0); //максимальное значение ускорения 16g при 16 битах(32767)
  float ay_temp = fabs(ay * 16.0 / 32767.0); //получаем значение от 0 до 16g в м/с^2
  float az_temp = fabs(az * 16.0 / 32767.0);
  /* Google:
     "Определение скорости и ускорения точки при координатном способе задания движения
     ... При этом в случае движения, происходящего в одной плоскости, во всех формулах
     должна быть отброшена проекция на ось z" */
  if (ax_temp > ay_temp & ay_temp > az_temp) {
    A = sqrt(/*ax_temp*ax_temp+*/ay_temp * ay_temp + az_temp * az_temp);
  }
  if (ay_temp > ax_temp & ax_temp > az_temp) {
    A = sqrt(ax_temp * ax_temp +/*ay_temp*ay_temp+*/az_temp * az_temp);
  }
  if (az_temp > ax_temp & ax_temp > ay_temp) {
    A = sqrt(ax_temp * ax_temp + ay_temp * ay_temp/*+az_temp*az_temp*/);
  }
  /* отладка
    Serial.print("A");
    Serial.print(ax_temp);
    Serial.print(";");
    Serial.print(ay_temp);
    Serial.print(";");
    Serial.print(az_temp);
    Serial.print(";");
    Serial.println();*/
  // Gyroscope
  int16_t gx = (Buf[8] << 8 | Buf[9]);
  int16_t gy = (Buf[10] << 8 | Buf[11]);
  int16_t gz = (Buf[12] << 8 | Buf[13]);

  //Magnetometer
  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  }
  while (!(ST1 & 0x01));
  // Read magnetometer data
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
  // Create 16 bits values from 8 bits data
  // Magnetometer
  int16_t mx = (Mag[3] << 8 | Mag[2]); //+=200
  int16_t my = (Mag[1] << 8 | Mag[0]); //-=70
  int16_t mz = (Mag[5] << 8 | Mag[4]); //-=700

  /*отработка логики*/
  if (Altitude > START_H) {
    Start = true;
  }
  SeparateF = analogRead(A6); //<800 - появился свет(теневой), значит ступень отделилась
  if (Start & SeparateF < SEPARATE_A) {
    Separate = true;
  }
  if (Separate & !Recovery) {
    RecoveryN = Time;
    Recovery = true;
  }
  if (((RecoveryN + TIMESERVO) > Time & Recovery & !Landing & TimeRecToLand < 7) || (Alt_temp_help_bool & !Landing)) { // TIMESERVO секунд крутим мотор
    digitalWrite(A3, HIGH);
    ++TimeRecToLand;/*переменная введена для тестирования в условиях комнаты.
    полет в этом случае на 2-3 метра и между Recovery и Landing не остается времени
    крутить серву. TimeRecToLand - делает необходимую задержку. Это не повлияет на
    работу в эксплуатации. delay() в данном случае использовать нельзя, он не позволит
    осуществлять передачу по радио во время выполнения своих задержек*/
  }
  else {
    digitalWrite(A3, LOW);
  }
  if (Recovery & !Landing & Altitude<8.0 & TimeRecToLand>2) {
    if ((Altitude - LandingH) < 4.0) {
      Landing = true;
    }
    LandingH = Altitude;
  }
  if (Alt_temp_help < Altitude) {
    Alt_temp_help = Altitude;
  }
  if (Alt_temp_help > Altitude & Alt_temp_help > 60 & Altitude < AltRec) {
    Alt_temp_help_bool = true;
    Recovery = true;
  }
  bool Recovery2021 = Recovery;
  if (/*Recovery & !Separate & */ Altitude >= 50) {
    Recovery2021 = false;
  }


  /*закончили с логикой*/
  /*Телеметрия по радиоканалу в соответствии с форматом:
    TeamID;TIme;Altitude;A;Start point;Separate point;Recovery point;Landing point \n*/
  Serial.print(TeamID);
  Serial.print(";");
  Serial.print(Time);
  Serial.print(";");
  Serial.print(Altitude);
  Serial.print(";");
  Serial.print(A);
  Serial.print(";");
  Serial.print(Start);
  Serial.print(";");
  Serial.print(Separate);
  Serial.print(";");
  Serial.print(Recovery2021);
  Serial.print(";");
  Serial.print(Landing);
  Serial.print(";");



  Serial.println();
  /*Интерфейс индикатора по сегментам в рабочем режиме*/
  digitalWrite(9, (int)(Battery / 50)); //заряд более 50%
  digitalWrite(8, VCCSV610); //питание на радиомодуль(подано через транзистор, управляется через А0)
  digitalWrite(7, WELL); //отвечает за microSD и HW612. система готова для запуска (радиомодуль проверять по принятым пакетам)
  digitalWrite(6, TESTING); //индикатор активен если система в режиме тестирования
  digitalWrite(5, Start); //активен, после фиксации старта ракеты
  digitalWrite(4, Separate); //активен, после фиксации разделения с ракетоносителем
  digitalWrite(3, Recovery2021); //активен, после активации системы спасения
  digitalWrite(2, Landing); //активен, после приземления
  /* запись телеметрии на карту памяти в соответствии с форматом:
     TeamID;TIme;Altitude;Ax;Ay;Az;Gy;Gx;Gz;Mx;My;Mz;Pressure;Temperature;Start
     point;Separate point;Recovery point;Landing point \n
  */
  if (dataFile) {
    dataFile.print(TeamID);
    dataFile.print(";");
    dataFile.print(Time);
    dataFile.print(";");
    dataFile.print(Altitude);
    dataFile.print(";");
    dataFile.print(ax_temp);
    dataFile.print(";");
    dataFile.print(ay_temp);
    dataFile.print(";");
    dataFile.print(az_temp);
    dataFile.print(";");
    dataFile.print(gx);
    dataFile.print(";");
    dataFile.print(gy);
    dataFile.print(";");
    dataFile.print(gz);
    dataFile.print(";");
    dataFile.print(mx);
    dataFile.print(";");
    dataFile.print(my);
    dataFile.print(";");
    dataFile.print(mz);
    dataFile.print(";");
    dataFile.print(Pressure);
    dataFile.print(";");
    dataFile.print(Temperature);
    dataFile.print(";");
    dataFile.print(Start);
    dataFile.print(";");
    dataFile.print(Separate);
    dataFile.print(";");
    dataFile.print(Recovery2021);
    dataFile.print(";");
    dataFile.print(Landing);
    dataFile.println("");
    dataFile.close();
    //WELL=true;
  }
  else {
    WELL = false;
  }
  delay(1000);
}
