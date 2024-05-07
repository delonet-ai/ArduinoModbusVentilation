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

                    //Initilize servo object for class Servo
Modbus bus;   
const int arraySize = 2;         //Столько же, сколько адресов             
const int EEPROM_ADDRESS = 0;// Адрес в EEPROM, где хранится массив
uint16_t modbus_array[arraySize];    //первоначально в массив записываем нулевые значения
//uint32_t tmr1;        //таймер  
//unsigned long sec = millis();; 

void setup()
{

  // Считываем массив из EEPROM
  for (int i = 0; i < arraySize; i++) {
    modbus_array[i] = EEPROM.read(EEPROM_ADDRESS + i);
  };

  //Servo0 setup
  servo0.attach(9, 450, 2200); //для  futaba s3003
  servo0.setSpeed(50);   // ограничить скорость
  servo0.setAccel(0.1);  	// установить ускорение (разгон и торможение)
  //Servo 0 end settup

  //Servo1 setup
  servo1.attach(8, 200, 2000); //для BMS-620
  servo1.setSpeed(50);   // ограничить скорость
  servo1.setAccel(0.1);  	// установить ускорение (разгон и торможение)
  //Servo 1 end settup

  //Servo2 setup
  servo2.attach(7, 450, 2250);
  servo2.setSpeed(50);   // ограничить скорость
  servo2.setAccel(0.1);  	// установить ускорение (разгон и торможение)
  //Servo 2 end settup

  //Servo3 setup
  servo3.attach(6, 450, 2250);
  servo3.setSpeed(50);   // ограничить скорость
  servo3.setAccel(0.1);  	// установить ускорение (разгон и торможение)
  //Servo 3 end settup

  //Servo4 setup
  servo4.attach(5, 450, 2250);
  servo4.setSpeed(50);   // ограничить скорость
  servo4.setAccel(0.1);  	// установить ускорение (разгон и торможение)
  //Servo 4 end settup
 
 bus = Modbus(128,1,10);          //адрес ведомого равен 128, используется связь через интерфейс RS-485 (вторая 1), 10 – номер контакта Arduino, к которому подключены контакты DE & RE модуля RS-485
  bus.begin(9600);                //Modbus slave baudrate at 9600 (скорость 9600 бод)
}
void loop()
{

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
//Вывод данных для дебага
// Serial.print("Channel 0-> ");
 //Serial.print(modbus_array[0]);
//Serial.println(""); 
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
   
//Запуск серв
servo0.tick();     
servo1.tick();
servo2.tick();
servo3.tick();
servo4.tick();

//Записываем данные регистров в значение переменной (умножаем 100 на 1.8 чтобы получить градусы)
   int pwms0 = modbus_array[0]*1.8; //Depends upon value in modbus_array[1] written by Master Modbus
   int pwms1 = modbus_array[1]*1.8; //Depends upon value in modbus_array[1] written by Master Modbus
   int pwms2 = modbus_array[2]*1.8; //Depends upon value in modbus_array[1] written by Master Modbus
   int pwms3 = modbus_array[3]*1.8; //Depends upon value in modbus_array[1] written by Master Modbus
   int pwms4 = modbus_array[4]*1.8; //Depends upon value in modbus_array[1] written by Master Modbus
  




//Попытка чтобы сервы попорачивались по очереди
//if (millis() >= sec +10000) {
 //servo0.setTargetDeg(pwms0); 
//Serial.print("Serv 0->");
//Serial.println(""); 
 //    sec = millis(); 
 // } else { }


//Поворачиваем сервы на установленный угол
 servo0.setTargetDeg(pwms0);  
 servo1.setTargetDeg(pwms1); 
 servo2.setTargetDeg(pwms2); 
 servo3.setTargetDeg(pwms3); 
 servo4.setTargetDeg(pwms4);    	

  }
