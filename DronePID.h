
class PID
{
  public:  

  PID () { error = 0; integral_error = 0; last_error = 0; k_prop = 0; k_integ = 0; k_deriv = 0; }
  PID (float p, float i, float d) { error = 0; integral_error = 0; last_error = 0; k_prop = p; k_integ = i; k_deriv = d; } 
  void NextOutput (float error);
  void SetParameters (float p, float i, float d) { k_prop = p; k_integ = i; k_deriv = d; return;}
  

  private:  
  float integral_error;
  float last_error;

  float k_prop; 
  float k_integ; 
  float k_deriv;
  
  const int sample_time = 100;
}
