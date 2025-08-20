#include <ServoSmooth.h>
#include <EEPROM.h>

// RS-485 Modbus Slave (Arduino UNO)
// Circuit Digest
#include <ModbusRtu.h>       // Library for using Modbus in Arduino


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

// --- EEPROM layout for servo targets ---
const int SERVO_COUNT = 10;
const int EEPROM_SERVO_BASE = 100;     // separate area to avoid Modbus bytes
const uint8_t EEPROM_MAGIC = 0xA5;     // marker for initialized storage

// Track last Modbus snapshot to update servos only on change
uint16_t last_mb[SERVO_COUNT] = {0};

// Helpers
void loadTargetsFromEEPROM();
void saveTargetsToEEPROM();
void applyTargetsToServos(bool setCurrentToo);
void restoreAll();
void resetServos();

                    //Initilize servo object for class Servo
Modbus bus;
const int arraySize = SERVO_COUNT;         // Количество Modbus регистров для сервоприводов
const int EEPROM_ADDRESS = 0;              // Адрес в EEPROM, где хранится массив
uint16_t modbus_array[arraySize] = {0};    // Текущие значения регистров Modbus

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

// Выводит текущие значения Modbus-регистров в Serial для отладки
void printModbusData() {
  Serial.print(F("MB:"));
  for (int i = 0; i < arraySize; i++) {
    Serial.print(' ');
    Serial.print(modbus_array[i]);
  }
  Serial.println();
}

void restoreAll() {
  for (int i = 0; i < SERVO_COUNT; i++) targets[i] = 0;
  saveTargetsToEEPROM();
  // также сбросим «модбас‑регистры» в EEPROM на 0
  for (int b = 0; b < arraySize; b++) EEPROM.update(EEPROM_ADDRESS + b, 0);
  applyTargetsToServos(false);
}

void resetServos() {
  for (int i = 0; i < SERVO_COUNT; i++) targets[i] = 0;
  saveTargetsToEEPROM();
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
  servo0.attach(8, 100, 2500); 
  servo0.setSpeed(50);   // ограничить скорость
  servo0.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 0 end settup

  //Servo1 setup
  servo1.attach(9, 100, 2500); 
  servo1.setSpeed(50);   // ограничить скорость
  servo1.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 1 end settup

  //Servo2 setup
  servo2.attach(10, 100, 2500);
  servo2.setSpeed(50);   // ограничить скорость
  servo2.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 2 end settup

  //Servo3 setup
  servo3.attach(11, 100, 2500);
  servo3.setSpeed(50);   // ограничить скорость
  servo3.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 3 end settup

  //Servo4 setup
  servo4.attach(12, 100, 2500);
  servo4.setSpeed(50);   // ограничить скорость
  servo4.setAccel(0.1);   // установить ускорение (разгон и торможение)
  //Servo 4 end settup
 
  //Servo5 setup (впуск A0)
  servo5.attach(A0, 100, 2500);
  servo5.setSpeed(50);
  servo5.setAccel(0.1);
  //Servo 5 end setup

  //Servo6 setup (впуск A1)
  servo6.attach(A1, 100, 2500);
  servo6.setSpeed(50);
  servo6.setAccel(0.1);
  //Servo 6 end setup

  //Servo7 setup (впуск A2)
  servo7.attach(A2, 100, 2500);
  servo7.setSpeed(50);
  servo7.setAccel(0.1);
  //Servo 7 end setup

  //Servo8 setup (впуск A3)
  servo8.attach(A3, 100, 2500);
  servo8.setSpeed(50);
  servo8.setAccel(0.1);
  //Servo 8 end setup

  //Servo9 setup (впуск A4)
  servo9.attach(A4, 100, 2500);
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

  bus.poll(modbus_array, arraySize);  // Receive/write values from master

  // Сохраняем массив регистров в EEPROM при изменении
  for (int b = 0; b < arraySize; b++) {
    if (EEPROM.read(EEPROM_ADDRESS + b) != modbus_array[b]) {
      EEPROM.update(EEPROM_ADDRESS + b, modbus_array[b]);
    }
  }

  // Нормализация значений 0..N → 0..180°
  auto toDeg = [](uint16_t v) -> int {
    if (v <= 100)   return (int)(v * 1.8);        // 0..100 → 0..180°
    if (v <= 255)   return (int)((v * 180) / 255); // 0..255 → 0..180°
    if (v <= 1023)  return (int)((v * 180) / 1023);// 10‑бит → 0..180°
    if (v <= 4095)  return (int)((v * 180) / 4095);// 12‑бит → 0..180°
    if (v <= 1000)  return (int)((v * 180) / 1000);// 0..1000 → 0..180°
    return 180;                                    // защита
  };

  // Обновляем цели только при изменении Modbus регистров
  bool mbChanged = false;
  for (int i = 0; i < arraySize; i++) {
    if (modbus_array[i] != last_mb[i]) { mbChanged = true; break; }
  }
  if (mbChanged) {
    printModbusData();            // показать свежие значения Modbus
    for (int i = 0; i < arraySize; i++) {
      last_mb[i] = modbus_array[i];
      targets[i] = toDeg(modbus_array[i]);
    }
    saveTargetsToEEPROM();
  }

  // Обновляем положения сервоприводов
  applyTargetsToServos(false);
  for (int i = 0; i < SERVO_COUNT; i++) {
    getServoPtr(i)->tick();
  }

}
