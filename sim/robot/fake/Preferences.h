// Preferences.h (fake) - flash storage kept in memory.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "Arduino.h"

class Preferences {
 public:
  bool begin(const char* ns, bool = false) { ns_ = ns; return true; }
  void end() {}
  bool clear() {
    for (auto it = store().begin(); it != store().end();) {
      it = it->first.rfind(ns_ + "/", 0) == 0 ? store().erase(it) : std::next(it);
    }
    return true;
  }
  size_t putUChar(const char* key, uint8_t v) { return putBytes(key, &v, 1); }
  uint8_t getUChar(const char* key, uint8_t def = 0) {
    auto it = store().find(ns_ + "/" + key);
    return it == store().end() || it->second.empty() ? def : it->second[0];
  }
  size_t putBytes(const char* key, const void* data, size_t n) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    store()[ns_ + "/" + key] = std::vector<uint8_t>(p, p + n);
    return n;
  }
  size_t getBytesLength(const char* key) {
    auto it = store().find(ns_ + "/" + key);
    return it == store().end() ? 0 : it->second.size();
  }
  size_t getBytes(const char* key, void* out, size_t n) {
    auto it = store().find(ns_ + "/" + key);
    if (it == store().end()) return 0;
    const size_t k = std::min(n, it->second.size());
    memcpy(out, it->second.data(), k);
    return k;
  }

 private:
  std::string ns_;
  static std::map<std::string, std::vector<uint8_t>>& store() {
    static std::map<std::string, std::vector<uint8_t>> s;
    return s;
  }
};
