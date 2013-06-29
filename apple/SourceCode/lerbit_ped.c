#include <math.h>
#include "lerbit_ped.h"

static uint32_t lerbit_ped_steps;
static float lerbit_ped_distance;

char stepflag;
float stride, avgstride, accel_dat[500];
float maxavg, minavg, accel_avg, velocity, displace;
float distance;
int steps;
 
static int32_t cycle_count, tot_samples, avgconst = 1, latency = 4, avglen = 8;
static float rssdat, newmax, newmin, oldavg, newavg, avgthresh=1.0;
static float walkfudge = 0.0249;


uint32_t lerbit_ped_get_steps(void)
{
  return lerbit_ped_steps;
}

uint32_t lerbit_ped_get_distance(void)
{
  return lerbit_ped_distance;
}

void lerbit_ped_init(void)
{
  stepflag = 2;
  maxavg = -10000.0;
  minavg = 10000.0;
  newmax = -10000.0;
  newmin = 10000.0;
  oldavg = 0.0;
  newavg = 0.0;
  cycle_count = 0;
  tot_samples = 0;
  steps = 0;
  distance = 0.0;
  accel_avg = 0.0;
  velocity = 0.0;
  displace = 0.0;
  avgstride = 0.0;
}

int8_t IsStep(float avg, float oldavg)
{
  // this function attempts to determine when a step is complete
  float step_thresh = 5.0; // used to prevent noise from fooling the algorithm
  if (stepflag == 2)
  {
    if (avg > (oldavg + step_thresh))
      stepflag = 1;
    if (avg < (oldavg - step_thresh))
      stepflag = 0;
    return 0;
  } // first time through this function
  if (stepflag == 1)
  {
    if ((maxavg > minavg) && (avg >
          ((maxavg+minavg)/2)) &&
        (oldavg < ((maxavg+minavg/2))))
      return 1;
    if (avg < (oldavg - step_thresh))
    {
      stepflag = 0;
      if (oldavg > maxavg)
        maxavg = oldavg;
    } // slope has turned down
    return 0;
  } // slope has been up
  if (stepflag == 0)
  {
    if (avg > (oldavg + step_thresh))
    {
      stepflag = 1;
      if (oldavg < minavg)
        minavg = oldavg;
    } // slope has turned up
    return 0;
  } // slope has been down
  return 0;
}

void lerbit_ped_monitor(lerbit_ped_acc_data_t ped_data_X, lerbit_ped_acc_data_t ped_data_Y)
{
  int32_t i;

  if (ped_data_X < 0)
    ped_data_X = -ped_data_X;

  if (ped_data_Y < 0)
    ped_data_Y = -ped_data_Y;

  if (tot_samples > 7) // subtract first sample in sliding boxcar avg
  {
    oldavg = newavg;
    newavg -= accel_dat[cycle_count - avglen];
  } 

  // get data from accelerometer

  rssdat = sqrt((float)(ped_data_X*ped_data_X + ped_data_Y*ped_data_Y)/16.0); // vector sum
  accel_dat[cycle_count] = rssdat; // place current sample data in buffer
  newavg += rssdat; // add new sample to sliding boxcar avg
  if(newavg > oldavg) {
    if(newavg - oldavg < avgthresh)
      newavg = oldavg;
  } else {
    if(oldavg - newavg < avgthresh)
      newavg = oldavg;
  }
  if (rssdat > newmax)
    newmax = rssdat;
  if (rssdat < newmin)
    newmin = rssdat;
  tot_samples++;
  cycle_count++; // increment count of samples in current step
  if (tot_samples > 8)
  {
    if (IsStep(newavg, oldavg))
    {
      for (i = latency; i < (cycle_count - latency); i++)
        accel_avg += accel_dat[i];
      accel_avg /= (cycle_count - avglen);
      for (i = latency; i < (cycle_count - latency); i++)
      {
        velocity += (accel_dat[i] - accel_avg);
        displace += velocity;
      } // create integration and double integration
      // calculate stride length
      stride = displace * (newmax - newmin) / (accel_avg - newmin);
      if(stride < 0) /* stride = abs(stride) */
        stride = -stride;
      stride = sqrt(stride);
      // use appropriate constant to get stride length
      stride *= walkfudge;
      // generate exponential average of stride length to smooth data
      if (steps < 2)
        avgstride = stride;
      else
        avgstride = ((avgconst-1)*avgstride + stride)/avgconst;
      steps++;
      lerbit_ped_steps = steps;
      distance += avgstride;
      /* global_distance = distance*30.48; */
      lerbit_ped_distance = distance*30.48;
      // need all data used in calculating newavg
      for (i = 0; i < avglen; i++)
        accel_dat[i] = accel_dat[cycle_count + i - avglen];
      cycle_count = avglen;
      newmax = -10000.0;
      newmin = 10000.0;
      maxavg = -10000.0;
      minavg = 10000.0;
      accel_avg = 0;
      velocity = 0;
      displace = 0;
    } // we have a new step
  } // enough samples to start checking for step (need at least 8)
}
