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
  int getError();
  void setEnabled(bool e);
  bool isEnabled();
  int getProportionalComponent();
  int getIntegralComponent();
  int getDerivativeComponent();
  void setMaxIntegralCumulation(int max);
  int getMaxIntegralCumulation();
  int getIntegralCumulation();

  void setInputBounded(bool bounded);
  bool isInputBounded();
  void setInputBounds(int lower, int upper);
  int getInputLowerBound();
  int getInputUpperBound();
  void setOutputBounded(bool bounded);
  bool isOutputBounded();
  void setOutputBounds(int lower, int upper);
  int getOutputLowerBound();
  int getOutputUpperBound();
  void setFeedbackWrapped(bool wrap);
  bool isFeedbackWrapped();
  void setFeedbackWrapBounds(int lower, int upper);
  int getFeedbackWrapLowerBound();
  int getFeedbackWrapUpperBound();

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
  bool enabled;
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
  int inputLowerBound;
  int inputUpperBound;
  bool outputBounded;
  int outputLowerBound;
  int outputUpperBound;
  bool feedbackWrapped;
  int feedbackWrapLowerBound;
  int feedbackWrapUpperBound;

  bool timeFunctionRegistered;
  int (*_pidSource)();
  void (*_pidOutput)(int output);
  unsigned long (*_getSystemTime)();
};
