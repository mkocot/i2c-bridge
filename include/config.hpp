#if 1
#if X_DEBUG || 1
// Base case: print single argument
template<typename T>
void debug_print(T&& arg) {
  Serial.print(arg);
}

// Recursive case: print argument and recurse for remaining arguments
template<typename T, typename... Args>
void debug_print(T&& arg, Args&&... args) {
  Serial.print(arg);
  debug_print(std::forward<Args>(args)...);
}

// Overload for println for the last argument
template<typename T>
void debug_println(T&& arg) {
  Serial.println(arg);
}

// Recursive case: print argument and recurse for remaining arguments
template<typename T, typename... Args>
void debug_println(T&& arg, Args&&... args) {
  Serial.print(arg);
  if constexpr (sizeof...(args) > 0) {
    Serial.print(" ");
  } else {
    Serial.println();
  }
  debug_println(std::forward<Args>(args)...);
}

#else

template<typename T>
void debug_print(T&& arg) {
}

template<typename T>
void debug_println(T&& arg) {
}

#endif
#endif