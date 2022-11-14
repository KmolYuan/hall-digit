#define ANALOG_PIN A0 // AO to A0
#define UPPER 600     // no magnet upper threshold
#define LOWER 400     // no magnet lower threshold
#define N 12          // number of magnets
#define RADIUS 0.065  // radius in meter
#define EPSILON 1.1920929E-7

using Time = unsigned long;

enum class Sign : char {
  Neutral = 0,
  Positive = 1,
  Negative = -1,
};

template <size_t SIZE = 4> class Buffer {
  static_assert(SIZE >= 4);
  size_t end = 0; // An exclusive terminal index
  Time time[SIZE]{};
  Sign sign[SIZE]{};
  float velocity[SIZE]{};
  float v_time[SIZE]{};

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
    // Calculate vecolcity
    if (sign[end] != Sign::Neutral) {
      // 0-1 <=> 2-3
      // 0-1 <=> 1-2
      if (sign[last] == Sign::Neutral) {
        vel_block((size_t[]){0, 1, 2, 3});
      } else {
        vel_block((size_t[]){0, 1, 1, 2});
      }
    } else if (sign[index(2)] == Sign::Neutral) {
      // 0-1 <=> 2-3 (neutral)
      vel_block((size_t[]){0, 1, 2, 3});
    } else {
      // 1-2 <=> 2-3 (neutral)
      vel_block((size_t[]){1, 2, 2, 3});
    }
    // Calculate acceleration
    {
      const float t = float(v_time[end] - v_time[last]);
      const float acc =
          t == 0. ? 0. : 1000. * (velocity[end] - velocity[last]) / t;
      Serial.println(String(v_time[end]) + " " + String(velocity[end]) + " " +
                     String(acc));
    }
    end += 1;
    if (end == SIZE)
      end = 0;
  }

  void vel_block(const size_t (&ind)[4]) {
    const float t1 = float(time[index(ind[0])] + time[index(ind[1])]) * 0.5;
    const float t2 = float(time[index(ind[2])] + time[index(ind[3])]) * 0.5;
    const float t = t1 - t2;
    velocity[end] = t == 0. ? 0. : 2000. * RADIUS * PI / N / t;
    v_time[end] = (t1 + t2) * 0.5;
  }

  // Negative index from the end, where `index(0) == end`.
  auto index(size_t i) const -> size_t {
    const int n = int(end) - int(i);
    return n < 0 ? SIZE + size_t(n) : size_t(n);
  }
};

void setup() { Serial.begin(9600); }

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
