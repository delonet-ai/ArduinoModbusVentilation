#include <ModbusRtu.h>
#include <ServoSmooth.h>
#include <EEPROM.h>

const uint8_t servoCount = 10;
const uint8_t servoPins[servoCount] = {8, 9, 10, 11, 12, A0, A1, A2, A3, A4};

ServoSmooth servos[servoCount];
Modbus bus;

const int EEPROM_ADDRESS = 0; // base address for stored servo angles
uint16_t modbus_array[servoCount];
uint16_t servoValues[servoCount];

void setup() {
  for (uint8_t i = 0; i < servoCount; i++) {
    servos[i].attach(servoPins[i], 100, 2500);
    servos[i].setSpeed(50);
    servos[i].setAccel(0.1);

    servoValues[i] = EEPROM.read(EEPROM_ADDRESS + i);
    modbus_array[i] = servoValues[i];
    servos[i].setTargetDeg(servoValues[i] * 1.8);
  }

  bus = Modbus(128, 1, 2); // slave address 128, RS-485 DE/RE on pin D2
  bus.begin(9600);
}

void loop() {
  bus.poll(modbus_array, servoCount);

  for (uint8_t i = 0; i < servoCount; i++) {
    if (modbus_array[i] != servoValues[i]) {
      servoValues[i] = modbus_array[i];
      EEPROM.update(EEPROM_ADDRESS + i, servoValues[i]);
      servos[i].setTargetDeg(servoValues[i] * 1.8);
    }
    servos[i].tick();
  }
}
