#ifndef DEBUG_H
#define DEBUG_H

// ---------------------------------------------------------------------------
// Debug output macros
// ---------------------------------------------------------------------------
// In the 'esp32dev' (debug) environment, DEBUG_MODE is defined via build_flags
// and these macros forward to Serial.  In the 'esp32dev_release' environment,
// DEBUG_MODE is not defined and all macros compile to nothing - no code is
// generated, no flash is consumed, Serial is not initialised.
//
// Usage:
//   DBG_PRINT("value=");
//   DBG_PRINTLN(someInt);
//   DBG_PRINTLN("my message");
// ---------------------------------------------------------------------------

#ifdef DEBUG_MODE
  #define DBG_INIT(baud)    Serial.begin(baud)
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DBG_INIT(baud)    do {} while (0)
  #define DBG_PRINT(...)    do {} while (0)
  #define DBG_PRINTLN(...)  do {} while (0)
#endif

#endif // DEBUG_H
