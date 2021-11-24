#ifndef FILTER_H
#define FILTER_H

// #define FIR_FILTER_LEN 16
#define FIR_FILTER_LEN 21

class FIR_Filter
{
  public:
    FIR_Filter();
    float update(float input);

  private:
    float convolution();
    float circularbuf[FIR_FILTER_LEN];
    static const float coeff[FIR_FILTER_LEN];
    int bufindx;  
    float fir_output; 
};


class IIR_Filter
{
  public:
    IIR_Filter(float alpha, float initial_val);
    float update(float input);

  private:
    float a;
    float b; 
    float prev; 
    float iir_output;
};



#endif 
