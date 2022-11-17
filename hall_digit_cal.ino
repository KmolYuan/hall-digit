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
#define N 2                // number of magnets
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
  Time time[SIZE]{};
  Sign sign[SIZE]{};
  Velocity v[SIZE]{};

public:
  void update(const Time t, const Sign s) {
    const size_t last = index(1);
    if (t == time[last])
      return;
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
    {
      // Calculate vecolcity
      if (sign[end] != Sign::Neutral) {
        // 0-1 <=> 2-3
        // 0-1 <=> 1-2
        if (sign[last] == Sign::Neutral) {
          v[end] = vel_block((size_t[]){0, 1, 2, 3});
        } else {
          v[end] = vel_block((size_t[]){0, 1, 1, 2});
        }
      } else if (sign[index(2)] != Sign::Neutral) {
        // 1-2 <=> 2-3 (neutral)
        v[end] = vel_block((size_t[]){1, 2, 2, 3});
      } else {
        goto end;
      }
      // Calculate acceleration
      const Velocity v1 = {(v[end].time + v[last].time) * 0.5,
                           (v[end].v + v[last].v) * 0.5};
      const Velocity v2 = {(v[index(2)].time + v[index(3)].time) * 0.5,
                           (v[index(2)].v + v[index(3)].v) * 0.5};
      const Time t = v1.time - v2.time;
      const float a = t == 0. ? 0. : 1000. * (v1.v - v2.v) / t;
      // Print
      const Velocity *vp = &v[end];
      Serial.println(String(vp->time) + " " + String(vp->v) + " " + String(a));
    }
  end:
    end += 1;
    if (end == SIZE)
      end = 0;
  }

  auto vel_block(const size_t (&ind)[4]) const -> Velocity {
    const Time t1 = time[index(ind[0])] + time[index(ind[1])];
    const Time t2 = time[index(ind[2])] + time[index(ind[3])];
    const float t = (t1 - t2) * 0.5;
    return {(t1 + t2) * 0.25, t == 0. ? 0. : 2000. * RADIUS * PI / N / t};
  }

  // Negative index from the end, where `index(0) == end`.
  auto index(size_t i) const -> size_t {
    const int n = int(end) - int(i);
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

  const Time t = millis();
  Sign s = Sign::Neutral;
  {
    int analog = analogRead(ANALOG_PIN);
    if (analog > UPPER)
      s = Sign::Positive;
    else if (analog < LOWER)
      s = Sign::Negative;
    // Serial.println(String(t) + " " + String(analog) + " " + String(int(s)));
  }
  buffer.update(t, s);
}
