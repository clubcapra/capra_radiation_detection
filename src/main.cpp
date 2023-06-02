/*
 * Project: Radiation Detector - STM32
 * Author: David Caron
 * Organization: CAPRA - ÉTS
 *
 * Description:
 * This code implements a radiation detector using an STM32 microcontroller. It calculates the Counts Per Minute (CPM)
 * and uSv/h values based on the number of falling edges detected from a Geiger-Muller tube. The CPM and uSv/h values
 * are calculated every 5 seconds then saved into a buffer. According to what it is being detected in the last 5 seconds,
 * it will act accordingly to what average it should make and send the data over ROS Serial.
 *
 * Pin assignment:
 * - Geiger Input Pin: PB4
 *
 * Connection Diagram:
 * [ Geiger-Muller Tube ] -- [ PB4: Geiger Input Pin ]
 * [ CANBUS ]             -- [ PB8: CANRX, PB9: CANTX ]
 * [ USB ]                -- [ PA11: USB-, PA12: USB+ ]
 */
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>

#define GEIGERPIN PB4
#define POLLING_TIME 5000
#define BUFFER_SIZE (60000 / POLLING_TIME)
#define VARIATION_THRESHOLD 72
#define MIN_IN_MICRO 60000000UL
#define DEBUG 1
#define DBL_TICK_TRESHOLD 100000 //treshold in microseconds

#define LOWPASS_GAIN 0.007 //LOWER MEANS MORE FILTERING

volatile unsigned long fallingEdgeCount = 0;
unsigned long startTime = 0;
unsigned long previousTime = 0;
unsigned long previousCount = 0;
const float conversionRate = 153.0; // Number from datasheet CPM->uSv/h
bool flagBuffer = 0;
float averageCPMRecalculated = 0;
float averageuSvPHRecalculated = 0;

// ROTATING BUFFER
int cpmBuffer[BUFFER_SIZE];
float usvPerHourBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// ROS SERIAL INITIALISATION
ros::NodeHandle nh;
std_msgs::Float64 radiation;
ros::Publisher pub1("radiation", &radiation);

void printDebug(unsigned long count, unsigned long elapsedTime, unsigned int cpm, float usvPerHour);
float calculateUsvPH(unsigned int cpm, float conversionRate);
void printDebugAverage(void);
float calculateAverageUntilThresholdExceeded(void);
double fir_low_pass(double input_sample);

#define N 91   // Number of filter coefficients

// Filter coefficients
const double h[N] = {
    0.001867519746044820,
    0.002275293677911463,
    0.002692845467883248,
    0.003119400211540369,
    0.003554151391578438,
    0.003996262853518041,
    0.004444870878876082,
    0.004899086349965267,
    0.005357997000181999,
    0.005820669743355830,
    0.006286153075466025,
    0.006753479541784039,
    0.007221668262275161,
    0.007689727507889525,
    0.008156657320192493,
    0.008621452166627776,
    0.009083103623574271,
    0.009540603079249766,
    0.009992944448431896,
    0.010439126890909214,
    0.010878157525543523,
    0.011309054131818317,
    0.011730847830767882,
    0.012142585737226980,
    0.012543333575412086,
    0.012932178249941664,
    0.013308230364524597,
    0.013670626680692492,
    0.014018532509122373,
    0.014351144026291085,
    0.014667690509420725,
    0.014967436482914990,
    0.015249683769748768,
    0.015513773441556675,
    0.015759087661469738,
    0.015985051414072162,
    0.016191134117190663,
    0.016376851110586923,
    0.016541765016997088,
    0.016685486971350718,
    0.016807677714403370,
    0.016908048547430839,
    0.016986362145057821,
    0.017042433223728004,
    0.017076129063764694,
    0.017087369883420026,
    0.017076129063764694,
    0.017042433223728004,
    0.016986362145057821,
    0.016908048547430839,
    0.016807677714403370,
    0.016685486971350718,
    0.016541765016997088,
    0.016376851110586923,
    0.016191134117190663,
    0.015985051414072162,
    0.015759087661469738,
    0.015513773441556675,
    0.015249683769748768,
    0.014967436482914990,
    0.014667690509420725,
    0.014351144026291085,
    0.014018532509122373,
    0.013670626680692492,
    0.013308230364524597,
    0.012932178249941664,
    0.012543333575412086,
    0.012142585737226980,
    0.011730847830767882,
    0.011309054131818317,
    0.010878157525543523,
    0.010439126890909214,
    0.009992944448431896,
    0.009540603079249766,
    0.009083103623574271,
    0.008621452166627776,
    0.008156657320192493,
    0.007689727507889525,
    0.007221668262275161,
    0.006753479541784039,
    0.006286153075466025,
    0.005820669743355830,
    0.005357997000181999,
    0.004899086349965267,
    0.004444870878876082,
    0.003996262853518041,
    0.003554151391578438,
    0.003119400211540369,
    0.002692845467883248,
    0.002275293677911463,
    0.001867519746044820
};

// Buffer to store input samples
double x[N] = {0};

long int last_pulse = 0;

float cpm = 0;

int debounce_flag = 0;

long int pulse_interval = 99999999;
float cpm_avg = 0;

void geigerInterrupt(){
  long int pulse_stamp = micros();
  long int pulse_interval_tmp = pulse_stamp - last_pulse;
  
  if(pulse_interval_tmp < DBL_TICK_TRESHOLD){
    if(debounce_flag > 3){
      pulse_interval = pulse_interval_tmp;
    }
    debounce_flag++;
  }
  else{
    pulse_interval = pulse_interval_tmp;
    debounce_flag = 0;
  }
  last_pulse = pulse_stamp;
}

void setup()
{
  if (DEBUG){
    Serial.begin(115200);
  }

  // GEIGER
  pinMode(GEIGERPIN, INPUT_FLOATING);
  attachInterrupt(digitalPinToInterrupt(GEIGERPIN), geigerInterrupt, FALLING);

  // TIMER
  startTime = millis();

  // ROS SETUP
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub1);
}



void loop()
{
  long int timestmp = micros();
  
  if(micros()- last_pulse > pulse_interval){
    pulse_interval = micros()- last_pulse;
  }

  float cpm_tmp = MIN_IN_MICRO/(float)pulse_interval;
  //cpm_avg = cpm_avg - LOWPASS_GAIN*(cpm_avg - cpm_tmp);
  cpm_avg = fir_low_pass(cpm_tmp);
  Serial.println(cpm_avg,10);
  // send the message via ROS Serial
  radiation.data = pulse_interval/1000.0;
  pub1.publish(&radiation);
  nh.spinOnce();

  while(micros() - timestmp < 100000){
  }
}

double fir_low_pass(double input_sample)
{
    // Shift the input samples in the buffer
    for (int i = N - 1; i > 0; i--)
    {
        x[i] = x[i - 1];
    }

    // Store the new input sample at the beginning of the buffer
    x[0] = input_sample;

    // Perform the filtering operation
    double output_sample = 0;
    for (int i = 0; i < N; i++)
    {
        output_sample += h[i] * x[i];
    }

    return output_sample;
}

void printDebug(unsigned long count, unsigned long elapsedTime, unsigned int cpm, float usvPerHour)
{
  // print current value
  Serial.print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\ncurrent on buffer: ");
  Serial.print(bufferIndex);
  Serial.print("\ncurrent CPM: ");
  Serial.print(cpm);
  Serial.print("\ncurrent uSv/h: ");
  Serial.print(usvPerHour);
  Serial.print("\nElapsed time: ");
  Serial.print(elapsedTime);

  // Print circulat buffers
  Serial.print("\nuSv/h : ");
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    Serial.print(" [");
    Serial.print(usvPerHourBuffer[i]);
    Serial.print("] ");
  }
  Serial.print("\nCPM: ");
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    Serial.print(" [");
    Serial.print(cpmBuffer[i]);
    Serial.print("] ");
  }
}

void printDebugAverage(void)
{
  // Print the average uSv/h in the circular buffer
  Serial.print("\nBuffer uSvPH is full: The average of it is : ");
  float sumuSvph = 0.0;
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    sumuSvph += usvPerHourBuffer[i];
  }
  float averageusvPerHour = sumuSvph / BUFFER_SIZE;
  Serial.print(averageusvPerHour);

  // Print the average CPM in the circular buffer
  Serial.print("\nBuffer CPM is full: The average of it is : ");
  float sumuCPM = 0.0;
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    sumuCPM += cpmBuffer[i];
  }
  float averageCPM = sumuCPM / BUFFER_SIZE;
  Serial.println(averageCPM);
}

float calculateUsvPH(unsigned int cpm, float conversionRate)
{
  float calculatedUSVPH;
  calculatedUSVPH = cpm / conversionRate; // Conversion rate taken from datasheet
  return calculatedUSVPH;
}

float calculateAverageUntilThresholdExceeded(void)
{

  bool thresholdExceeded = false;
  int sum = 0;
  int i = 0;
  int currentValue = cpmBuffer[bufferIndex];
  int delta;
  float newCPMValue;

  // Look through the entire circular buffer if threshhold isn't exceded
  while (!thresholdExceeded && i < BUFFER_SIZE)
  {
    // Calculate the index in reverse order
    int index = (bufferIndex - i + BUFFER_SIZE) % BUFFER_SIZE;

    delta = fabs(currentValue - cpmBuffer[index]);

    // Check if the current value checked is within the threshhold
    if ((delta <= VARIATION_THRESHOLD) && (!thresholdExceeded))
    {
      sum += cpmBuffer[index];
    }
    else
    {
      thresholdExceeded = true;
    }
    i++;
  }

  // calculate the average of what is within the threshhold
  newCPMValue = (float)sum / i;

  return newCPMValue;
}
