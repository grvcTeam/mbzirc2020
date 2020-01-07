#ifndef TIMER_H
#define TIMER_H

class Timer {
public:

  Timer(uint16_t period_in_ms, uint8_t max_period_multiple = 10) {
    period_in_ms_ = period_in_ms;
    max_period_multiple_ = max_period_multiple;
    last_loop_time_ms_ = 0;
    period_counter_ = 0;
  }

  void begin() {
    last_loop_time_ms_ = millis();
  }

  bool checkPeriodMultiple(uint8_t mult) {
    return (period_counter_ % mult)  == 0;
  }

  void sleep() {
    update();

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

  void update() {
    if (period_counter_ >= max_period_multiple_-1) {
      period_counter_ = 0;
    } else {
      period_counter_++;
    }
  }

  uint16_t period_in_ms_;
  uint8_t max_period_multiple_;
  unsigned long last_loop_time_ms_;
  uint16_t period_counter_;
};

#endif  // TIMER_H

