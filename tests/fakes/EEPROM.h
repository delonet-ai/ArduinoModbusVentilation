#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

class EEPROMClass {
 public:
  static constexpr size_t kSize = 1024;

  EEPROMClass() { reset(); }

  void reset() { memset(data_, 0xFF, sizeof(data_)); }

  uint8_t read(int address) const { return data_[address]; }

  void update(int address, uint8_t value) { data_[address] = value; }

  template <typename T>
  T &get(int address, T &value) const {
    memcpy(&value, data_ + address, sizeof(T));
    return value;
  }

  template <typename T>
  const T &put(int address, const T &value) {
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
    for (size_t i = 0; i < sizeof(T); ++i) {
      update(address + i, bytes[i]);
    }
    return value;
  }

 private:
  uint8_t data_[kSize];
};

extern EEPROMClass EEPROM;
