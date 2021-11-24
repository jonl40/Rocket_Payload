#include "Filter.h"
#include "Arduino.h"


FIR_Filter::FIR_Filter()
{
  // init circularbuf vals to 0
  // copy FILTER_TAPS to coeff 
  for (int i=0; i<FIR_FILTER_LEN; i++)
    circularbuf[i] = 0; 

}

/*
const float FIR_Filter::coeff[FIR_FILTER_LEN] = { -0.032906f,
                                                  -0.0052635f, 
                                                  -0.0068811f, 
                                                  0.0f, 
                                                  0.0254209f, 
                                                  0.0724719f, 
                                                  0.1311260f, 
                                                  0.1805961f, 
                                                  0.20f, 
                                                  0.1805961f, 
                                                  0.1311260f, 
                                                  0.0724719f,
                                                  0.0254209f, 
                                                  0.0f, 
                                                  -0.0068811f, 
                                                  -0.0052635f};
*/

const float FIR_Filter::coeff[FIR_FILTER_LEN] = {  -0.046453312283160735,
                                                  -0.061832087078289695,
                                                  0.05925585766100568,
                                                  0.0025034623437773787,
                                                  -0.1328568724848226,
                                                  0.27095145118629244,
                                                  0.6695944748490683,
                                                  0.27095145118629244,
                                                  -0.1328568724848226,
                                                  0.0025034623437773787,
                                                  0.05925585766100568,
                                                  -0.061832087078289695,
                                                  -0.046453312283160735};

float FIR_Filter::update(float input)
{
  // update buffer with sensor data 
  circularbuf[bufindx] = input; 
  bufindx++;

  // wrap around if indx points out of bounds 
  if(bufindx >= FIR_FILTER_LEN)
      bufindx = 0;

  return FIR_Filter::convolution();
}


float FIR_Filter::convolution()
{
  fir_output = 0; 
  int indx = bufindx; 

  for (int j=0; j<FIR_FILTER_LEN; j++)
  {
    // wrap around 
    if (indx > 0)
      indx--; 
    else
      indx = FIR_FILTER_LEN - 1;
    
    fir_output += coeff[j] * circularbuf[indx];
  }
  
  return fir_output;
}


// first order IIR filter 
IIR_Filter::IIR_Filter(float alpha, float initial_val)
{
  a = 1-alpha;
  b = alpha; 
  prev = initial_val;

  // check stability 
  if (abs(a) >= 1)
  {
    Serial.println("absolute val of 'a' must be less than 1 for IIR to be stable");
    Serial.println("Setting alpha val to 0.5");
    a = 0.5f;
    b = 0.5f;
  }
}


// first order IIR filter
// y[n] = (1-alpha)x[n] + (alpha)y[n-1] 
// y[n] = (a)x[n]       + (b)y[n-1] 
float IIR_Filter::update(float input)
{
  iir_output = a * input + b * prev; 
  prev = iir_output;
  return iir_output; 
}
