#include <Adafruit_PWMServoDriver.h>
#include <ServoDriverSmooth.h>
#include <ServoSmooth.h>
#include <smoothUtil.h>
#include <EEPROM.h>

//RS-485 Modbus Slave (Arduino UNO)
//Circuit Digest
#include<ModbusRtu.h>       //Library for using Modbus in Arduino
#include <Servo.h>          //Library for using Servo Motor


ServoSmooth servo0; 
ServoSmooth servo1;   
ServoSmooth servo2;   
ServoSmooth servo3;   
ServoSmooth servo4;     

ServoSmooth servo5;
ServoSmooth servo6;
ServoSmooth servo7;
ServoSmooth servo8;
ServoSmooth servo9;

// Целевые углы для 10 серв: 0..4 — выпуск (D8–D12), 5..9 — впуск (A0–A4)
int targets[10] = {0};

// Таймер для компактного вывода
unsigned long lastPrintMs = 0;

void handleSerial();

// --- EEPROM layout for servo targets ---
const int SERVO_COUNT = 10;
const int EEPROM_SERVO_BASE = 100;     // separate area to avoid Modbus bytes
const uint8_t EEPROM_MAGIC = 0xA5;     // marker for initialized storage

// Track last Modbus snapshot to update E-group only on change
uint16_t last_mb[5] = {0,0,0,0,0};
bool mb_inited = false;

// Helpers
void loadTargetsFromEEPROM();
void saveTargetsToEEPROM();
void applyTargetsToServos(bool setCurrentToo);
void restoreAll();

                    //Initilize servo object for class Servo
Modbus bus;   
const int arraySize = 5;         //Столько же, сколько адресов             
const int EEPROM_ADDRESS = 0;// Адрес в EEPROM, где хранится массив
uint16_t modbus_array[arraySize];    //первоначально в массив записываем нулевые значения
//uint32_t tmr1;        //таймер  
//unsigned long sec = millis();; 

void handleSerial() {
  if (!Serial.available()) return;
String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  cmd.toUpperCase();

  if (cmd == F("RESTORE")) {
    restoreAll();
    Serial.println(F("OK RESTORE"));
    return;
  }

  if (cmd.startsWith("ALL")) {
    int val = cmd.substring(3).toInt();
    val = constrain(val, 0, 180);
    for (int i = 0; i < 10; i++) targets[i] = val;
    saveTargetsToEEPROM();
    Serial.println(F("OK ALL"));
    return;
  }

  int idx = -1;
  int angle = -1;
  char type = cmd.charAt(0);

  int p = 1;
  while (p < cmd.length() && !isDigit(cmd[p])) p++;
  if (p < cmd.length()) {
    int p2 = p;
    while (p2 < cmd.length() && isDigit(cmd[p2])) p2++;
    idx = cmd.substring(p, p2).toInt();
    while (p2 < cmd.length() && !isDigit(cmd[p2])) p2++;
    if (p2 < cmd.length()) {
      angle = cmd.substring(p2).toInt();
    }
  }

  if (angle < 0) return;
  angle = constrain(angle, 0, 180);

  int gi = -1;
  if (type == 'E') {
    if (idx >= 0 && idx < 5) gi = idx;
  } else if (type == 'I') {
    if (idx >= 0 && idx < 5) gi = 5 + idx;
  } else if (type == 'S') {
    if (idx >= 0 && idx < 10) gi = idx;
  }

  if (gi >= 0) {
    targets[gi] = angle;
    saveTargetsToEEPROM();
    Serial.print(F("OK S"));
    Serial.print(gi);
    Serial.print(F("="));
    Serial.println(angle);
  }
}

 


// --- helpers implementation ---
static ServoSmooth* getServoPtr(int i) {
  switch (i) {
    case 0: return &servo0; case 1: return &servo1; case 2: return &servo2; case 3: return &servo3; case 4: return &servo4;
    case 5: return &servo5; case 6: return &servo6; case 7: return &servo7; case 8: return &servo8; case 9: return &servo9;
  }
  return &servo0;
}

void loadTargetsFromEEPROM() {
  uint8_t m = EEPROM.read(EEPROM_SERVO_BASE);
  if (m != EEPROM_MAGIC) {
    // init defaults
    for (int i = 0; i < SERVO_COUNT; i++) targets[i] = 0;
    EEPROM.update(EEPROM_SERVO_BASE, EEPROM_MAGIC);
    for (int i = 0; i < SERVO_COUNT; i++) EEPROM.update(EEPROM_SERVO_BASE + 1 + i, (uint8_t)targets[i]);
  } else {
    for (int i = 0; i < SERVO_COUNT; i++) {
      int v = EEPROM.read(EEPROM_SERVO_BASE + 1 + i);
      targets[i] = constrain(v, 0, 180);
    }
  }
}

void saveTargetsToEEPROM() {
  EEPROM.update(EEPROM_SERVO_BASE, EEPROM_MAGIC);
  for (int i = 0; i < SERVO_COUNT; i++) EEPROM.update(EEPROM_SERVO_BASE + 1 + i, (uint8_t)constrain(targets[i],0,180));
}

void applyTargetsToServos(bool setCurrentToo) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    ServoSmooth* s = getServoPtr(i);
    if (setCurrentToo) s->setCurrentDeg(targets[i]);
    s->setTargetDeg(targets[i]);
  }
}

void restoreAll() {
  for (int i = 0; i < SERVO_COUNT; i++) targets[i] = 0;
  saveTargetsToEEPROM();
  // также сбросим «модбас‑регистры» в EEPROM на 0
  for (int b = 0; b < arraySize; b++) EEPROM.update(EEPROM_ADDRESS + b, 0);
  applyTargetsToServos(false);
}

void setup(){

  // Считываем массив из EEPROM
  for (int i = 0; i < arraySize; i++) {
    modbus_array[i] = EEPROM.read(EEPROM_ADDRESS + i);
    last_mb[i] = modbus_array[i];
  }
  // загрузим стартовые углы сервоприводов из EEPROM
  loadTargetsFromEEPROM();

  Serial.begin(115200);

  //Servo0 setup
  servo0.attach(8, 450, 2200); //для  futaba s3003
  servo0.setSpeed(50);   // ограничить скорость
  servo0.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 0 end settup

  //Servo1 setup
  servo1.attach(9, 200, 2000); //для BMS-620
  servo1.setSpeed(50);   // ограничить скорость
  servo1.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 1 end settup

  //Servo2 setup
  servo2.attach(10, 450, 2250);
  servo2.setSpeed(50);   // ограничить скорость
  servo2.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 2 end settup

  //Servo3 setup
  servo3.attach(11, 450, 2250);
  servo3.setSpeed(50);   // ограничить скорость
  servo3.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 3 end settup

  //Servo4 setup
  servo4.attach(12, 450, 2250);
  servo4.setSpeed(50);   // ограничить скорость
  servo4.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 4 end settup
 
  //Servo5 setup (впуск A0)
  servo5.attach(A0, 450, 2250);
  servo5.setSpeed(50);
  servo5.setAccel(0.1);
  //Servo 5 end setup

  //Servo6 setup (впуск A1)
  servo6.attach(A1, 450, 2250);
  servo6.setSpeed(50);
  servo6.setAccel(0.1);
  //Servo 6 end setup

  //Servo7 setup (впуск A2)
  servo7.attach(A2, 450, 2250);
  servo7.setSpeed(50);
  servo7.setAccel(0.1);
  //Servo 7 end setup

  //Servo8 setup (впуск A3)
  servo8.attach(A3, 450, 2250);
  servo8.setSpeed(50);
  servo8.setAccel(0.1);
  //Servo 8 end setup

  //Servo9 setup (впуск A4)
  servo9.attach(A4, 450, 2250);
  servo9.setSpeed(50);
  servo9.setAccel(0.1);
  //Servo 9 end setup

  // Применим стартовые значения без рывка: текущая=цель
  applyTargetsToServos(true);

  bus = Modbus(128,1,10);          //адрес ведомого равен 12...та Arduino, к которому подключены контакты DE & RE модуля RS-485
  bus.begin(9600);                //Modbus slave baudrate at 9600 (скорость 9600 бод)
}
void loop()
{

  handleSerial();

   bus.poll(modbus_array,sizeof(modbus_array)/sizeof(modbus_array[0]));  //Used to receive or write value from Master 
    // Сохраняем массив в EEPROM
for (int b = 0; b < arraySize; b++)
{

if (EEPROM.read(EEPROM_ADDRESS +b) != modbus_array[b])
{
  EEPROM.update(EEPROM_ADDRESS + b, modbus_array[b]);
Serial.print(modbus_array[b]);
Serial.println(""); 
}
else {};
} //for

 //else
 // Serial.print("Channel 0-> ");
  //Serial.print(modbus_array[0]);
 // Serial.println(""); 

  //   Serial.print("Channel 1-> ");
  // Serial.print(modbus_array[1]);
 // Serial.println(""); 

 //   Serial.print("Channel 2-> ");
  //Serial.print(modbus_array[2]);
 // Serial.println(""); 

 //   Serial.print("Channel 3-> ");
 // Serial.print(modbus_array[3]);
 // Serial.println(""); 

   // Serial.print("Channel 4-> ");
  //Serial.print(modbus_array[4]);
  //Serial.println(""); 
 //Вывод данных для дебага
   
 
//Записываем данные регистров в значение переменной (нормализация 0..N → 0..180)
  auto toDeg = [](uint16_t v) -> int {
    if (v <= 100)   return (int)(v * 1.8);        // 0..100 → 0..180°
    if (v <= 255)   return (int)((v * 180) / 255); // 0..255 → 0..180°
    if (v <= 1023)  return (int)((v * 180) / 1023);// 10‑бит → 0..180°
    if (v <= 4095)  return (int)((v * 180) / 4095);// 12‑бит → 0..180°
    if (v <= 1000)  return (int)((v * 180) / 1000);// 0..1000 → 0..180°
    return 180;                                    // защита
  };

  // Обновляем цели для E-группы ТОЛЬКО при изменении регистров Modbus
  bool mbChanged = false;
  for (int i = 0; i < arraySize; i++) {
    if (modbus_array[i] != last_mb[i]) { mbChanged = true; break; }
  }
  if (mbChanged) {
    for (int i = 0; i < arraySize; i++) last_mb[i] = modbus_array[i];
    int pwms0 = toDeg(modbus_array[0]);
    int pwms1 = toDeg(modbus_array[1]);
    int pwms2 = toDeg(modbus_array[2]);
    int pwms3 = toDeg(modbus_array[3]);
    int pwms4 = toDeg(modbus_array[4]);
    targets[0] = pwms0; targets[1] = pwms1; targets[2] = pwms2; targets[3] = pwms3; targets[4] = pwms4;
    saveTargetsToEEPROM();
  }

 //Попытка чтобы сервы попорачивались по очереди
//sec = modbus_array[0];
//if (sec> 900)  sec = millis();

//if (millis() - sec < 5000) {
//    servo0.setTargetDeg(pwms4); 
//   Serial.print("1-> ");
//   Serial.print(pwms4);
//   Serial.print("/");
//   Serial.print(modbus_array[4]);
//   Serial.print("/");
// Serial.println(""); 
 //    sec = millis(); 
 // } else { }


//Поворачиваем сервы на установленный угол
 servo0.setTargetDeg(targets[0]);
 servo1.setTargetDeg(targets[1]);
 servo2.setTargetDeg(targets[2]);
 servo3.setTargetDeg(targets[3]);
 servo4.setTargetDeg(targets[4]);    	
 servo5.setTargetDeg(targets[5]);
 servo6.setTargetDeg(targets[6]);
 servo7.setTargetDeg(targets[7]);
 servo8.setTargetDeg(targets[8]);
 servo9.setTargetDeg(targets[9]);

 // Теперь один тик после установки целей, чтобы не было "рывка" на старте
 servo0.tick();
 servo1.tick();
 servo2.tick();
 servo3.tick();
 servo4.tick();
 servo5.tick();
 servo6.tick();
 servo7.tick();
 servo8.tick();
 servo9.tick();

  // Компактный вывод позиций всех серв в одной строке
  if (millis() - lastPrintMs >= 500) {
    lastPrintMs = millis();
    Serial.print(F("POS E:"));
    for (int i = 0; i < 5; i++) {
      Serial.print(targets[i]);
      if (i < 4) Serial.print(',');
    }
    Serial.print(F(" I:"));
    for (int i = 5; i < 10; i++) {
      Serial.print(targets[i]);
      if (i < 9) Serial.print(',');
    }
    Serial.println();
  }

}
