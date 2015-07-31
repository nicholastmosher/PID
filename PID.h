#ifndef PID_H
#define PID_H
#endif

class PIDController
{
public:

  PIDController(double p, double i, double d, int (*getFeedback)(), void (*onUpdate)(int output));
  void tick();
  void setTarget(int t);
  int getTarget();
  int getOutput();
  int getFeedback();
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
  void setPIDSource(int (*getFeedback)());
  void setPIDOutput(void (*onUpdate)(int output));
  void registerTimeInput(unsigned long (*getSystemTime)());
  void unregisterTimeInput();

private:

  double _p;
  double _i;
  double _d;
  int target;
  int output;
  int currentFeedback;
  int lastFeedback;
  int lastError;
  long currentTime;
  long lastTime;
  int integralCumulation;
  int maxCumulation;
  bool inputBounded;
  bool outputBounded;
  int inputLowerBound;
  int inputUpperBound;
  int outputLowerBound;
  int outputUpperBound;
  bool timeInputRegistered;
  int error();
  long deltaTime();
  int (*_getFeedback)();
  void (*_onUpdate)(int output);
  unsigned long (*_getSystemTime)();
};
