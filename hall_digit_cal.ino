#define SERVO

#ifdef SERVO
#include <Event.h>
#include <Servo.h>
#include <Timer.h>
#define SERVO_PIN A1
#endif

#define ANALOG_PIN A0      // AO to A0
#define UPPER 600          // no magnet upper threshold
#define LOWER 400          // no magnet lower threshold
#define N 4                // number of magnets
#define RADIUS (180. / PI) // radius in meter

using Time = unsigned long;

enum class Sign : char {
  Neutral = 0,
  Positive = 1,
  Negative = -1,
};

struct Velocity {
  float time;
  float v;
};

template <size_t SIZE = 4> class Buffer {
  static_assert(SIZE >= 4);
  size_t end = 0; // An exclusive terminal index
  size_t v_end = 0;
  Time time[SIZE]{};
  Sign sign[SIZE]{};
  Velocity v[SIZE]{};

public:
  void update(const Time t, const int analog) {
    Sign s = Sign::Neutral;
    if (analog > UPPER)
      s = Sign::Positive;
    else if (analog < LOWER)
      s = Sign::Negative;
    const size_t last = index(1);
    if (t == time[last])
      return;
    // Serial.println(String(t) + " " + String(analog));
    // Update last (or append new signal)
    if (s == sign[last]) {
      time[last] = t;
      return;
    }
    // Add missing signal
    if (sign[last] == Sign::Neutral && sign[index(2)] == s) {
      if (s == Sign::Positive)
        sign[last] = Sign::Negative;
      else if (s == Sign::Negative)
        sign[last] = Sign::Positive;
    }
    // Append
    time[end] = t;
    sign[end] = s;
    // Calculate vecolcity
    if (sign[end] != Sign::Neutral) {
      // 0-1 <=> 2-3
      // 0-1 <=> 1-2
      if (sign[last] == Sign::Neutral) {
        v[v_end] = vel_block((size_t[]){0, 1, 2, 3});
      } else {
        v[v_end] = vel_block((size_t[]){0, 1, 1, 2});
      }
    } else {
      goto hell;
    }
    { // Calculate acceleration
      const Velocity v1 = {(v[v_end].time + v[v_index(1)].time) * 0.5,
                           (v[v_end].v + v[v_index(1)].v) * 0.5};
      const Velocity v2 = {(v[v_index(2)].time + v[v_index(3)].time) * 0.5,
                           (v[v_index(2)].v + v[v_index(3)].v) * 0.5};
      const Time t = v1.time - v2.time;
      const float a = t == 0. ? 0. : 1000. * (v1.v - v2.v) / t;
      // Print
      const Velocity *vp = &v[v_end];
      Serial.println(String(vp->time) + " " + String(analog) + " " +
                     String(vp->v) + " " + String(a));
      v_end += 1;
      if (v_end == SIZE)
        v_end = 0;
    }
  hell:
    end += 1;
    if (end == SIZE)
      end = 0;
  }

  auto vel_block(const size_t (&ind)[4]) const -> Velocity {
    const float t1 = (time[index(ind[0])] + time[index(ind[1])]) * 0.5;
    const float t2 = (time[index(ind[2])] + time[index(ind[3])]) * 0.5;
    const float t = t1 - t2;
    return {(t1 + t2) * 0.5, t == 0. ? 0. : 2000. * RADIUS * PI / N / t};
  }

  // Negative index from the end, where `index(0) == end`.
  auto index(size_t i) const -> size_t {
    const int n = int(end) - int(i);
    return n < 0 ? SIZE + size_t(n) : size_t(n);
  }
  auto v_index(size_t i) const -> size_t {
    const int n = int(v_end) - int(i);
    return n < 0 ? SIZE + size_t(n) : size_t(n);
  }
};

#ifdef SERVO
Servo servo;
Timer timer;

void rot_servo() {
  static int pos = 0;
  servo.write(pos++);
}
#endif

void setup() {
#ifdef SERVO
  servo.attach(SERVO_PIN);
  timer.every(1000, rot_servo);
#endif
  Serial.begin(9600);
}

void loop() {
  static Buffer<4> buffer;
  buffer.update(millis(), analogRead(ANALOG_PIN));
}
