#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// ---------- variáveis PID legado
double SETPOINT = 25;  // Set your desired temperature here

double Kc = 1;
double Ti = 1;
double Td = 1;

double min_PID = 0;
double max_PID = 0;

double pid_output = 0;
int k = 1;

double error[2] = {0, 0}; 

// --------- anti-windup
double integrative[2] = {0, 0}; 
double tau = 1; 
double min_output = 1; 
double max_output = 1; 
double limiter_output = 1; 

// --------- filtro derivativo
double cut_off_freq = 1; 
double derivative_filter[2] = {0, 0};

// --------- setup PWM 
double duty_output = 0;
double max_duty = 255;  // for ESP32, the duty cycle ranges from 0 to 255
double min_duty = 0; 

// ---------- tempo de amostragem 
double Ts = 0.2; 
unsigned long previousMillis = 0;

// Pin for PWM output
const int pwmPin = 5;  // Change this pin to your desired PWM pin on ESP32

// DHT sensor setup
#define DHTPIN 14     // Digital pin connected to the DHT sensor (change accordingly)
#define DHTTYPE DHT11  // DHT 11 

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();

  // initialize PWM
  ledcSetup(0, 5000, 8);  // channel 0, 5000 Hz, 8-bit resolution
  ledcAttachPin(pwmPin, 0);

  // initialize variables
  unsigned long currentMillis = millis();
  previousMillis = currentMillis;
}

void loop() {
  // verifica quanto tempo passou desde a função tic 
  unsigned long currentMillis = millis();
  unsigned long elapsedTime = currentMillis - previousMillis;

  if (elapsedTime > Ts * 1000) {
    double sensor_measure = dht.readTemperature();  // Read temperature from DHT sensor

    error[k] = sensor_measure - SETPOINT;

    double proporcional = Kc * error[k]; 
    integrative[k] = integrative[k-1] + Kc * (Ts / Ti) * error[k-1]; 

    //  -------- Anti-windup
    integrative[k] = integrative[k] + (Ts / tau) * (limiter_output - pid_output);

    // --------- filtro derivativo
    derivative_filter[k] = (derivative_filter[k-1] + cut_off_freq * error[k] - cut_off_freq * error[k-1]) / cut_off_freq * Ts; 
    double derivative = Kc * (Td / Ts) * (error[k] - error[k-1]) * derivative_filter[k];

    pid_output = proporcional + integrative[k] + derivative;

    // ------- Limitador
    if (pid_output <= min_output) {
      limiter_output = min_output; 
    } else if (pid_output >= max_output) {
      limiter_output = max_output;
    } else {
      limiter_output = pid_output;
    }

    duty_output = (pid_output - min_PID) / (max_PID - min_PID) * (max_duty - min_duty) + min_duty;
    duty_output = constrain(duty_output, min_duty, max_duty);

    // Output PWM
    ledcWrite(0, duty_output);

    // Print the values for debugging
    Serial.print("Temperature: ");
    Serial.print(sensor_measure);
    Serial.print(" | Error: ");
    Serial.print(error[k]);
    Serial.print(" | PID Output: ");
    Serial.print(pid_output);
    Serial.print(" | Duty Cycle: ");
    Serial.println(duty_output);

    previousMillis = currentMillis;
    k = (k + 1) % 2;  // Switch between 0 and 1

    delay(50);  // Optional delay for stability
  }
}
