#ifndef I2C_BRIDGE_CONFIG_H
#define I2C_BRIDGE_CONFIG_H

#include <Arduino.h>

#define X_DEBUG 1

#if X_DEBUG
extern Stream &SerialDebug;
// Base case: print single argument
template <typename T>
void debug_print(T &&arg)
{
  SerialDebug.print(arg);
}

// Recursive case: print argument and recurse for remaining arguments
template <typename T, typename... Args>
void debug_print(T &&arg, Args &&...args)
{
  SerialDebug.print(arg);
  debug_print(std::forward<Args>(args)...);
}

// Overload for println for the last argument
template <typename T>
void debug_println(T &&arg)
{
  SerialDebug.println(arg);
}

// Recursive case: print argument and recurse for remaining arguments
template <typename T, typename... Args>
void debug_println(T &&arg, Args &&...args)
{
  SerialDebug.print(arg);

  if constexpr (sizeof...(args) == 0)
  {
    SerialDebug.println();
  }

  debug_println(std::forward<Args>(args)...);
}

#else

#if 1
// Shaving few bytes by using empty defines instead empty functions
#define debug_print(T...) do {} while(0)
#define debug_println(T...) debug_print(...)
#else
template <typename... T>
void debug_print(T &&...arg)
{
}

template <typename... T>
void debug_println(T &&...arg)
{
}
#endif

#endif

#endif