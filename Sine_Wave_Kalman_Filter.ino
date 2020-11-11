
#include <math.h>
#define AMPLITUDE 128
#define FREQUENCY 1000
#define PI 3.14159
#define PHASE 0
#define NOISE AMPLITUDE/64
#define PLOTTING_FREQUENCY 25
#define DELAY 50
volatile unsigned long i = 0;             // Initalize loop counter | Using for loops crashes the program for some reason 
float R = 40;                             // Noise covariance (Should be 10) | Higher R --> Filters more, but slower
float H = 1.00;                           // Measurement map scalar
float Q = 10;                             // Initial estimated covariance
float P = 0;                              // Intial error covariance (must be 0)
float U_hat = 0;                          // Initial estimated state (Assume we don't know)
float K = 0;                              // Initial Kalman Gain | More gain --> Slower filter


double KALMAN(double U)
{
  K = P * H / (H * P * H + R);            // Update Kalman Gain
  U_hat = U_hat + K * (U - H * U_hat);    // Update Estimated

  P = (1 - K * H) * P + Q;                // Update error covariance
  
  return U_hat;                           // Return filtered signal to main loop
}


void setup() 
{
  // Establish serial communication 
  Serial.begin(115200);
  // Labels for Serial Plotter Legend
  Serial.println("NOISY,FILTERED");
}

void loop() 
{
  // Resets counter
  if ( i > PLOTTING_FREQUENCY ){i = 0;}

  // Arrays HAVE to be intialized here to get expected results
  double FILTERED_SIGNAL[i];
  double NOISY_SIGNAL[i];

  // Generate Sine wave with noise 
  NOISY_SIGNAL[i] = AMPLITUDE * sin(2 * PI * FREQUENCY * i + PHASE) + random(-NOISE, NOISE);

  Serial.print(NOISY_SIGNAL[i]);Serial.print(',');

  // As of 11/10 this for loop is needed to print out filtered signal. Using a counter alone will crash the program for some reason
  for (int x = 0; x == 0; x++)
  {
    FILTERED_SIGNAL[x] = KALMAN(NOISY_SIGNAL[i]);
    Serial.println(FILTERED_SIGNAL[x]);

  }
  // Iterate counter and add 50 ms delay
  i = i + 1;
  delay(DELAY);
}
