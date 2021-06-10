#include <math.h>
#define AMPLITUDE 64
#define FREQUENCY 1000
#define PI 3.14159
#define PHASE 0
#define NOISE AMPLITUDE/32
#define PLOTTING_FREQUENCY 30
#define ITERATION_COUNT 30

float R_1 = 10;                             // Noise covariance (Should be 10) | Higher R --> Filters more, but slower
float R_2 = 20;                             // Noise covariance (Should be 10) | Higher R --> Filters more, but slower
float R_3 = 30;                             // Noise covariance (Should be 10) | Higher R --> Filters more, but slower
float R_4 = 40;                             // Noise covariance (Should be 10) | Higher R --> Filters more, but slower

float H = 1.00;                           // Measurement map scalar
float Q = 10;                             // Initial estimated covariance
float P = 0;                              // Intial error covariance (must be 0)
float U_hat_1, U_hat_2,
      U_hat_3, U_hat_4 = 0;               // Initial estimated state (Assume we don't know)
float K = 0.00;                           // Initial Kalman Gain | More gain --> Slower filter
float FILTERED_SIGNAL_1, FILTERED_SIGNAL_2,
      FILTERED_SIGNAL_3, FILTERED_SIGNAL_4,
      NOISY_SIGNAL;

float filter_signal_1(float &raw_signal)
{
  K = P * H / (H * P * H + R_1);                     // Update Kalman Gain
  U_hat_1 = U_hat_1 + K * (raw_signal - H * U_hat_1);    // Update Estimated

  P = (1 - K * H) * P + Q;                         // Update error covariance

  return U_hat_1;                                    // Return filtered signal to main loop
}
float filter_signal_2(float &raw_signal)
{
  K = P * H / (H * P * H + R_2);                     // Update Kalman Gain
  U_hat_2 = U_hat_2 + K * (raw_signal - H * U_hat_2);    // Update Estimated

  P = (1 - K * H) * P + Q;                         // Update error covariance

  return U_hat_2;                                    // Return filtered signal to main loop
}
float filter_signal_3(float &raw_signal)
{
  K = P * H / (H * P * H + R_3);                     // Update Kalman Gain
  U_hat_3 = U_hat_3 + K * (raw_signal - H * U_hat_3);    // Update Estimated

  P = (1 - K * H) * P + Q;                         // Update error covariance

  return U_hat_3;                                    // Return filtered signal to main loop
}
float filter_signal_4(float &raw_signal)
{
  K = P * H / (H * P * H + R_4);                     // Update Kalman Gain
  U_hat_4 = U_hat_4 + K * (raw_signal - H * U_hat_4);    // Update Estimated

  P = (1 - K * H) * P + Q;                         // Update error covariance

  return U_hat_4;                                    // Return filtered signal to main loop
}
void setup()
{
  // Establish serial communication
  Serial.begin(115200);
  // Labels for Serial Plotter Legend
  Serial.println("NOISY,FILTERED_1,FILTERED_2,FILTERED_3,FILTERED_4");

  //Number of waveforms
  for (int i = 0; i < 2; i++)
  {
    //Can change amplitude of Sinewave by increasing iteration count
    for (int i = 0; i < ITERATION_COUNT; i++)
    {
      //Generate Sine wave with noise
      NOISY_SIGNAL = AMPLITUDE * sin(2 * PI * FREQUENCY * i + PHASE) + random(-NOISE, NOISE);

      //Filter noisy Sine wave
      FILTERED_SIGNAL_1 = filter_signal_1(NOISY_SIGNAL);
      FILTERED_SIGNAL_2 = filter_signal_2(NOISY_SIGNAL);
      FILTERED_SIGNAL_3 = filter_signal_3(NOISY_SIGNAL);
      FILTERED_SIGNAL_4 = filter_signal_4(NOISY_SIGNAL);

      Serial.print(NOISY_SIGNAL); 
      Serial.print(',');
      Serial.print(FILTERED_SIGNAL_1);
      Serial.print(',');
      Serial.print(FILTERED_SIGNAL_2);
      Serial.print(',');       
      Serial.print(FILTERED_SIGNAL_3);
      Serial.print(',');      
      Serial.println(FILTERED_SIGNAL_4);
    }
  }
}
void loop()
{}
