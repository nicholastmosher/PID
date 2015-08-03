#include "PID.h"

/**
 * A PID Controller is a method of system control in which a correctional output
 * is generated to guide the system toward a desired setpoint (aka target).
 * The PID Controller calculates the output based on the following factors:
 *
 *    Gains (proportional, integral, and derivative)
 *    Target
 *    Feedback
 *
 * The gain values act as multipliers for their corresponding components of PID
 * (more detail later).  The target is the value which the system strives to
 * reach by manipulating the output.  The feedback is the system's actual
 * position or status in regards to the physical world.
 * Another important term in PID is "error", which refers to the difference
 * between the target and the feedback.
 *
 * Each of the three components of PID contributes a unique behavior to the
 * system.
 *
 *    The Proportional component introduces a linear relationship between the
 *    error (target minus feedback) and the output.  This means that as the
 *    feedback grows further away from the target, the output grows
 *    proportionally stronger.
 *
 *        Proportional component = (P Gain) * (target - feedback)
 *
 *    The Integral component is designed to give a very precise approach of the
 *    feedback to the target.  Depending on the scale of the physical system
 *    and the precision of feedback (e.g. sensors), the proportional component
 *    alone is likely not sufficient to provide adequate power (e.g. to motors)
 *    to guide the system in regards to small-scale correction.  The Integral
 *    component integrates the error of the system (target - feedback) over
 *    time.  If the system reaches a point where it is close but not exactly
 *    on top of the target, the integration will slowly build until it is
 *    powerful enough to overcome static resistances and move the system
 *    preciesly to the target.
 *
 *        Integral component = (I Gain) * Integral of error over time
 *
 *          *In this implementation, Integral is calculated with a running
 *          summation of the system's error, updated at each tick.
 *
 *    The Derivative component measures the rate of change of the feedback.
 *    It can reduce the strength of the output if the feedback is approaching
 *    the target too quickly or if the feedback is moving away from the target.
 *
 *        Derivative component = (D Gain) * ((error - lastError) / time - lastTime)
 */
PIDController::PIDController(double p, double i, double d, int (*pidSource)(), void (*pidOutput)(int output))
{
  _p = p;
  _i = i;
  _d = d;
  target = 0;
  output = 0;
  currentFeedback = 0;
  lastFeedback = 0;
  lastError = 0;
  currentTime = 0L;
  lastTime = 0L;
  integralCumulation = 0;
  maxCumulation = 30000;
  inputBounded = false;
  outputBounded = false;
  inputLowerBound = 0;
  inputUpperBound = 0;
  outputBounded = false;
  outputLowerBound = 0;
  outputUpperBound = 0;
  timeFunctionRegistered = false;
  _pidSource = pidSource;
  _pidOutput = pidOutput;
}

/**
 * This method uses the established function pointers to retrieve system
 * feedback, calculate the PID output, and deliver the correction value
 * to the parent of this PIDController.  This method should be run as
 * fast as the source of the feedback in order to provide the highest
 * resolution of control (for example, to be placed in the loop() method).
 */
void PIDController::tick()
{
  //Retrieve system feedback from user callback.
  currentFeedback = _pidSource();

  //Apply input bounds if necessary.
  if(inputBounded)
  {
    if(currentFeedback > inputUpperBound) currentFeedback = inputUpperBound;
    if(currentFeedback < inputLowerBound) currentFeedback = inputLowerBound;
  }

  //Calculate the error between the feedback and the target.
  error = target - currentFeedback;

  //If we have a registered way to retrieve the system time, use time in PID calculations.
  if(timeFunctionRegistered)
  {
    //Retrieve system time
    currentTime = _getSystemTime();

    //Calculate time since last tick() cycle.
    long deltaTime = currentTime - lastTime;

    //Calculate the integral of the feedback data since last cycle.
    int cycleIntegral = (lastError + error / 2) * deltaTime;

    //Add this cycle's integral to the integral cumulation.
    integralCumulation += cycleIntegral;

    //Calculate the slope of the line with data from the current and last cycles.
    cycleDerivative = (error - lastError) / deltaTime;

    //Save time data for next iteration.
    lastTime = currentTime;
  }
  //If we have no way to retrieve system time, estimate calculations.
  else
  {
    integralCumulation += error;
    cycleDerivative = (error - lastError);
  }

  //Prevent the integral cumulation from becoming overwhelmingly huge.
  if(integralCumulation > maxCumulation) integralCumulation = maxCumulation;
  if(integralCumulation < -maxCumulation) integralCumulation = -maxCumulation;

  //Calculate the system output based on data and PID gains.
  output = (int) ((error * _p) + (integralCumulation * _i) + (cycleDerivative * _d));

  //Save a record of this iteration's data.
  lastFeedback = currentFeedback;
  lastError = error;

  //Trim the output to the bounds if needed.
  if(outputBounded)
  {
    if(output > outputUpperBound) output = outputUpperBound;
    if(output < outputLowerBound) output = outputLowerBound;
  }

  _pidOutput(output);
}

/**
 * Sets the target of this PIDController.  This system will generate
 * correction outputs indended to guide the feedback variable (such
 * as position, velocity, etc.) toward the established target.
 */
void PIDController::setTarget(int t)
{
  target = t;
}

/**
 * Returns the current target of this PIDController.
 * @return The current target of this PIDController.
 */
int PIDController::getTarget()
{
  return target;
}

/**
 * Returns the latest output generated by this PIDController.  This value is
 * also delivered to the parent systems via the PIDOutput function pointer
 * provided in the constructor of this PIDController.
 * @return The latest output generated by this PIDController.
 */
int PIDController::getOutput()
{
  return output;
}

/**
 * Returns the last read feedback of this PIDController.
 */
int PIDController::getFeedback()
{
  return currentFeedback;
}

/**
 * Returns the value that the Proportional component is contributing to the output.
 * @return The value that the Proportional component is contributing to the output.
 */
int PIDController::getProportionalComponent()
{
  return (int) (error * _p);
}

/**
 * Returns the value that the Integral component is contributing to the output.
 * @return The value that the Integral component is contributing to the output.
 */
int PIDController::getIntegralComponent()
{
  return (int) (integralCumulation * _i);
}

/**
 * Returns the value that the Derivative component is contributing to the output.
 * @return The value that the Derivative component is contributing to the output.
 */
int PIDController::getDerivativeComponent()
{
  return (int) (cycleDerivative * _d);
}

/**
 * Sets the maximum value that the integral cumulation can reach.
 * @param max The maximum value of the integral cumulation.
 */
void PIDController::setMaxIntegralCumulation(int max)
{
  //If the new max value is less than 0, invert to make positive.
  if(max < 0)
  {
    max = -max;
  }

  //If the new max is not more than 1 then the cumulation is useless.
  if(max > 1)
  {
    maxCumulation = max;
  }
}

/**
 * Returns the maximum value that the integral value can cumulate to.
 * @return The maximum value that the integral value can cumulate to.
 */
int PIDController::getMaxIntegralCumulation()
{
  return maxCumulation;
}

/**
 * Returns the current cumulative integral value in this PIDController.
 * @return The current cumulative integral value in this PIDController.
 */
int PIDController::getIntegralCumulation()
{
  return integralCumulation;
}

/**
 * Enables or disables bounds on the input.  Bounds limit the upper and
 * lower values that this PIDController will ever accept as input.
 * Outlying values will be trimmed to the upper or lower bound as necessary.
 * @param bounded True to enable input bounds, False to disable.
 */
void PIDController::setInputBounded(bool bounded)
{
  inputBounded = bounded;
}

/**
 * Enables or disables bounds on the output.  Bounds limit the upper and lower
 * values that this PIDController will ever generate as output.
 * @param bounded True to enable output bounds, False to disable.
 */
void PIDController::setOutputBounded(bool bounded)
{
  outputBounded = bounded;
}

/**
 * Returns whether the input of this PIDController is being bounded.
 * @return True if the input of this PIDController is being bounded.
 */
bool PIDController::isInputBounded()
{
  return inputBounded;
}

/**
 * Returns whether the output of this PIDController is being bounded.
 * @return True if the output of this PIDController is being bounded.
 */
bool PIDController::isOutputBounded()
{
  return outputBounded;
}

/**
 * Sets bounds which limit the lower and upper extremes that this PIDController
 * accepts as inputs.  Outliers are trimmed to the lower and upper bounds.
 * Setting input bounds automatically enables input bounds.
 * @param lower The lower input bound.
 * @param upper The upper input bound.
 */
void PIDController::setInputBounds(int lower, int upper)
{
  if(upper > lower)
  {
    inputBounded = true;
    inputUpperBound = upper;
    inputLowerBound = lower;
  }
}

/**
 * Sets bounds which limit the lower and upper extremes that this PIDController
 * will ever generate as output.  Setting output bounds automatically enables
 * output bounds.
 * @param lower The lower output bound.
 * @param upper The upper output bound.
 */
void PIDController::setOutputBounds(int lower, int upper)
{
  if(upper > lower)
  {
    outputBounded = true;
    outputLowerBound = lower;
    outputUpperBound = upper;
  }
}

/**
 * Returns the lower input bound of this PIDController.
 * @return The lower input bound of this PIDController.
 */
int PIDController::getInputLowerBound()
{
  return inputLowerBound;
}

/**
 * Returns the upper input bound of this PIDController.
 * @return The upper input bound of this PIDController.
 */
int PIDController::getInputUpperBound()
{
  return inputUpperBound;
}

/**
 * Returns the lower output bound of this PIDController.
 * @return The lower output bound of this PIDController.
 */
int PIDController::getOutputLowerBound()
{
  return outputLowerBound;
}

/**
 * Returns the upper output bound of this PIDController.
 * @return The upper output bound of this PIDController.
 */
int PIDController::getOutputUpperBound()
{
  return outputUpperBound;
}

/**
 * Sets new values for all PID Gains.
 * @param p The new proportional gain.
 * @param i The new integral gain.
 * @param d The new derivative gain.
 */
void PIDController::setPID(double p, double i, double d)
{
  _p = p;
  _i = i;
  _d = d;
}

/**
 * Sets a new value for the proportional gain.
 * @param p The new proportional gain.
 */
void PIDController::setP(double p)
{
  _p = p;
}

/**
 * Sets a new value for the integral gain.
 * @param i The new integral gain.
 */
void PIDController::setI(double i)
{
  _i = i;
}

/**
 * Sets a new value for the derivative gain.
 * @param d The new derivative gain.
 */
void PIDController::setD(double d)
{
  _d = d;
}

/**
 * Returns the proportional gain.
 * @return The proportional gain.
 */
double PIDController::getP()
{
  return _p;
}

/**
 * Returns the integral gain.
 * @return The integral gain.
 */
double PIDController::getI()
{
  return _i;
}

/**
 * Returns the derivative gain.
 * @return The derivative gain.
 */
double PIDController::getD()
{
  return _d;
}

/**
 * Sets the function pointer to the PID Source.  A PID Source
 * is a function which returns a value to be used as the PIDController's
 * control feedback.  This value can be a reading from a sensor or other
 * data source that contains information regarding the system's actual
 * state.
 * Below is an example of using a PIDSource:
 *
 *    int pidSource()
 *    {
 *        return mySensor.getValue();
 *    }
 *    myPIDController.setPIDSource(pidSource);
 *
 * @param (*getFeedback) A function pointer that retrieves system feedback.
 */
void PIDController::setPIDSource(int (*pidSource)())
{
  _pidSource = pidSource;
}

/**
 * Sets the function pointer to the PID Output.  A PID Output
 * is a function which delivers a value to the parent system in order to guide
 * the system based on the PID loop's result.  This value can be delivered
 * directly to motors, to a variable that directs steering, or other means of
 * influencing the system.
 * Below is an example of using a PIDOutput:
 *
 *    void pidOutput(int output)
 *    {
 *        myMotor.write(output);
 *    }
 *    myPIDController.setPIDOutput(pidOutput);
 *
 * @param (*onUpdate) A function pointer that delivers system output.
 */
void PIDController::setPIDOutput(void (*pidOutput)(int output))
{
  _pidOutput = pidOutput;
}

/**
 * Use this to add a hook into the PID Controller that allows it to
 * read the system time no matter what platform this library is run
 * on.  Though developed for an arduino project, there is no code
 * tying this library to the arduino project.  To use the arduino
 * clock, however, register the time-getting function (millis())
 * like this:
 *
 *    myPIDController.registerTimeInput(millis);
 *    *Note that in this example, millis has no parentheses.
 *
 * @param (*getSystemTime) Pointer to a function that returns system time.
 */
void PIDController::registerTimeFunction(unsigned long (*getSystemTime)())
{
  _getSystemTime = getSystemTime;
  timeFunctionRegistered = true;
}
