// Simple class that helps to keep frequency loop constant, 
// sleeping for the remaining time
// freal@us.es
#ifndef TIMER_H
#define TIMER_H

class Timer {
public:

  Timer(uint16_t period_in_ms) {
    period_in_ms_ = period_in_ms;
    last_loop_time_ms_ = 0;
  }

  void begin() {
    last_loop_time_ms_ = millis();
  }

  void sleep() {
    unsigned long elapsed_ms;
    unsigned long current_time_ms = millis();
    if (current_time_ms < last_loop_time_ms_) {
      // millis (32 bits) has overflown (happens every ~50 days)
      elapsed_ms = 2^32 - 1 - last_loop_time_ms_;
      elapsed_ms += current_time_ms;
    } else {
      elapsed_ms = current_time_ms - last_loop_time_ms_;
    }

    if (elapsed_ms < period_in_ms_) {
      delay(period_in_ms_ - elapsed_ms);
    }

    last_loop_time_ms_ = millis();
  }

protected:

  uint16_t period_in_ms_;
  unsigned long last_loop_time_ms_;
};

#endif  // TIMER_H

