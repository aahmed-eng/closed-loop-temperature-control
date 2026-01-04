// temp_control_pi.ino

#include <DHT.h>

// ---------- USER SETTINGS ----------
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Sampling period (DHT11 limit)
const unsigned long Ts_ms = 1000; // 1 second
const float Ts = Ts_ms / 1000.0f;

// Motor driver pins (adjust if you used other pins)
const int motorEN = 5; // PWM pin (0..255)
const int motorIN1 = 4;
const int motorIN2 = 3;

// Temperature setpoint (°C)
float setpointC = 20.0f;

// EMA (first-order IIR) filter cutoff and alpha (computed in setup)
float fc = 0.5f; // cutoff frequency (Hz) for EMA
float alpha = 0.0f; // computed from fc and Ts in setup

// PI controller gains
float Kp = 20.0f;   // proportional gain (PWM per °C)
float Ki = 5.0f;    // integral gain (PWM per °C per s)

// Actuator limits (PWM)
const int u_min = 0;
const int u_max = 255;

// Integrator state (initial)
float I = 0.0f;

// Optional rate limiting (set high to disable)
const float max_delta_per_step = 255.0f;

// ---------- END USER SETTINGS ----------

unsigned long t_prev = 0;
float last_u = 0.0f;
bool initialized = false;

// EMA state
float ema_state = 0.0f;
bool ema_initialized = false;

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Motor driver initialisation: set fixed direction, control speed via EN PWM
  pinMode(motorEN, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  digitalWrite(motorIN1, HIGH); // forward
  digitalWrite(motorIN2, LOW);

  // Compute EMA alpha from chosen cutoff fc and sample period Ts:
  // alpha = (2π fc Ts) / (1 + 2π fc Ts)
  float two_pi_fc_ts = TWO_PI * fc * Ts;
  alpha = two_pi_fc_ts / (1.0f + two_pi_fc_ts);

  // Initialise state
  t_prev = millis();

  // Print CSV header for Python to parse
  Serial.println("time_ms,setpointC,temp_raw,temp_filt,pwm,e,I");
}

void loop() {
  unsigned long t = millis();
  if (t - t_prev >= Ts_ms) {
    t_prev += Ts_ms;

    // Read temperature. DHT11 returns NaN on error; skip those samples.
    float temp = dht.readTemperature(); // Celsius
    if (isnan(temp)) {
      // Log an error line and continue
      Serial.print(millis()); Serial.println(",ERR,,,0,0,0");
      return;
    }

    // EMA filter (runs on Arduino to reduce jitter before control)
    if (!ema_initialized) {
      ema_state = temp;
      ema_initialized = true;
    } else {
      ema_state = alpha * temp + (1.0f - alpha) * ema_state;
    }
    float temp_filt = ema_state;

    // Compute error (setpoint - measurement)
    float e = - (setpointC - temp_filt);

    // Candidate integrator update (discrete sum e * Ts)
    float I_candidate = I + e * Ts;

    // Unclamped PMW
    float u_unclamped = Kp * e + Ki * I_candidate;

    // Clamp actuator to PWM limits
    float u = constrain(u_unclamped, u_min, u_max);

    // Anti-windup: clamp integrator so controller output respects actuator limits
    if (Ki != 0.0f) {
      float I_min = (u_min - Kp * e) / Ki;
      float I_max = (u_max - Kp * e) / Ki;
      I = constrain(I_candidate, min(I_min, I_max), max(I_min, I_max));
    } else {
      I = I_candidate;
    }

    // Optional rate limiting (protects mechanical/electrical components)
    float delta = u - last_u;
    if (abs(delta) > max_delta_per_step) {
      if (delta > 0) u = last_u + max_delta_per_step;
      else u = last_u - max_delta_per_step;
    }

    // Apply PWM to motor (0..255)
    analogWrite(motorEN, (int)round(u));
    last_u = u;

    // Emit CSV log: time_ms,setpointC,temp_raw,temp_filt,pwm,e,I
    Serial.print(millis()); Serial.print(',');
    Serial.print(setpointC); Serial.print(',');
    Serial.print(temp); Serial.print(',');
    Serial.print(temp_filt); Serial.print(',');
    Serial.print((int)round(u)); Serial.print(',');
    Serial.print(e); Serial.print(',');
    Serial.println(I);
  }

  // Non-blocking serial commands for runtime tuning:
  // - "SP <value>" to set setpoint (°C)
  // - "Kp <value>" to set Kp
  // - "Ki <value>" to set Ki
  // - "FC <value>" to set EMA cutoff (Hz)
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s.length() == 0) return;
    if (s.startsWith("SP ")) {
      float v = s.substring(3).toFloat();
      if (v < 5) v = 5; if (v > 60) v = 60;
      // update setpoint (integrator unchanged)
      setpointC = v;
      Serial.print("SP->"); Serial.println(v);
    } else if (s.startsWith("Kp ")) {
      Kp = s.substring(3).toFloat(); Serial.print("Kp->"); Serial.println(Kp);
    } else if (s.startsWith("Ki ")) {
      Ki = s.substring(3).toFloat(); Serial.print("Ki->"); Serial.println(Ki);
    } else if (s.startsWith("FC ")) {
      fc = s.substring(3).toFloat();
      float two_pi_fc_ts = TWO_PI * fc * Ts;
      alpha = two_pi_fc_ts / (1.0f + two_pi_fc_ts);
      Serial.print("fc->"); Serial.print(fc); Serial.print(" alpha->"); Serial.println(alpha);
    } else {
      Serial.print("Unknown command: "); Serial.println(s);
    }
  }
}
