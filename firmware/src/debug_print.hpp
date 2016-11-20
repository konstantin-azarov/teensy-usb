#ifndef __DEBUG_PRINT__HPP__
#define __DEBUG_PRINT__HPP__

#include "Arduino.h"

struct EndlMarker {};

template <class T>
struct Hex {
  Hex(T _n) : n(_n) { };
  T n;
};

template <class T>
Hex<T> hex(T t) { return Hex<T>(t); }

class DebugPrinter {
  public:
    template <class T> DebugPrinter& operator << (const T& v) {
      Serial.print(v);
      return *this;
    }

    template <class T>
    DebugPrinter& operator << (const Hex<T>& h) {
      Serial.print(h.n, HEX);
      return *this;
    }

    DebugPrinter& operator << (const EndlMarker&) {
      Serial.println();
      return *this;
    }
};

extern EndlMarker endl;
extern DebugPrinter debug_printer;

#if DEBUG_PRINT

#define DBG_LOG(system) debug_printer << #system ": "

#else

#define DBG_LOG(system) if (0) debug_printer

#endif

#endif

