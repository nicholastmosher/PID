#ifndef PID_H
#define PID_H
#endif

class PIDController
{
public:
  PIDController(double p, double i, double d, int (*pidSource)(), void (*pidOutput)(int output));
  void tick();
  void setTarget(int t);
  int getTarget();
  int getOutput();
  int getFeedback();
  int getProportionalComponent();
  void setMaxIntegralCumulation(int max);
  int getMaxIntegralCumulation();
  int getIntegralCumulation();
  int getIntegralComponent();
  int getDerivativeComponent();
  void setInputBounded(bool bounded);
  void setOutputBounded(bool bounded);
  bool isInputBounded();
  bool isOutputBounded();
  void setInputBounds(int lower, int upper);
  void setOutputBounds(int lower, int upper);
  int getInputLowerBound();
  int getInputUpperBound();
  int getOutputLowerBound();
  int getOutputUpperBound();
  void setPID(double p, double i, double d);
  void setP(double p);
  void setI(double i);
  void setD(double d);
  double getP();
  double getI();
  double getD();
  void setPIDSource(int (*pidSource)());
  void setPIDOutput(void (*pidOutput)(int output));
  void registerTimeFunction(unsigned long (*getSystemTime)());

private:

  double _p;
  double _i;
  double _d;
  int target;
  int output;
  int currentFeedback;
  int lastFeedback;
  int error;
  int lastError;
  long currentTime;
  long lastTime;
  int integralCumulation;
  int maxCumulation;
  int cycleDerivative;
  bool inputBounded;
  bool outputBounded;
  int inputLowerBound;
  int inputUpperBound;
  int outputLowerBound;
  int outputUpperBound;
  bool timeFunctionRegistered;
  int (*_pidSource)();
  void (*_pidOutput)(int output);
  unsigned long (*_getSystemTime)();
};
