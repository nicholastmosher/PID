# PID
A simple PID library for use with Arduino or other C++ applications.

A PID Controller is a method of system control in which a correctional output
is generated to guide the system toward a desired setpoint (aka `target`).
The `PIDController` calculates the `output` based on the following factors:

 * `Gains` (proportional, integral, and derivative)
 * `Target`
 * `Feedback`

 The `gains` act as multipliers for their corresponding components of PID.

  The `target` is a user-specified value which the system
strives to reach by manipulating the `output`.

  The `feedback` is the system's actual position or status in regards to the
physical world.  This can be a position read by an encoder, an orientation from
an accelerometer, etc. Something important to note is that the <i>unit</i> of
the system matches the unit of the `feedback`, and the `PIDController` will try
to maintain the value of the `target` in terms of the `feedback`.  For example:

  * Assuming a `feedback` read from a robot's position on a number line, the
  `target` would represent the desired position and the `PIDController` would
  use the `output` to maneuver the robot to that position.
  * Assuming a `feedback` reading the angular rate of a gyroscope, the `target`
  would represent a desired angular rate and the `PIDController` would use the
  `output` to maintain a rotation matching that rate.

    For each application of PID, the user must understand the desired behavior
    of the system and provide a method of `feedback` appropriate to achieve the
    goal (see `PIDSource`).

  Another important term in PID is `error`, which refers to the difference
between the target and the feedback.

Each of the three components of PID contributes a unique behavior to the
system.

### Proportional

The `Proportional` component introduces a linear relationship between the
`error` (target minus feedback) and the `output`.  This means that as the
`feedback` grows further away from the `target`, the `output` grows
proportionally stronger.

```
Proportional component = (P Gain) * (error)
                       = (P Gain) * (target - feedback)
```

### Integral

The `Integral` component is designed to give a very precise approach of the
`feedback` to the `target`.  Depending on the scale of the physical system
and the precision of feedback (e.g. sensors), the proportional component
alone is likely not sufficient to provide adequate power (e.g. to motors)
to guide the system in regards to small-scale correction.  The `Integral`
component integrates the `error` of the system (target - feedback) over
time.  If the system reaches a point where it is close to - but not exactly
on top of - the target, the integration will slowly build until it is
powerful enough to overcome static resistances and move the system
precisely to the target.

```
Integral component = (I Gain) * Integral of error over time
```

In this implementation, Integral is calculated with a running summation of the system's error, updated at each `tick()`.

### Derivative

The `Derivative` component measures the rate of change of the `feedback`.
It can reduce the strength of the `output` if 1) the `feedback` is approaching
the `target` too quickly or 2) if the `feedback` is moving away from the
`target`.

    Derivative component = (D Gain) * ((change in error) / (change in time))
                         = (D Gain) * ((error - lastError) / (time - lastTime))

### PID Equation

Now that we know what each component of PID contributes, the `output` of a
`PIDController` can be nicely summed up with:

```
PID Output = Proportional component + Integral component + Derivative component
```

# Library Features

## Platform Independence

This library was built with versatility in mind.  Though the project it was
built for is Arduino, no Arduino-specific function calls are made.  Instead,
function pointers provide hooks for gathering necessary inputs, such as system
time.

### PID Source, Output, and other Hooks

Since one of the primary focuses in this library is platform independence, there
are several instances where function pointers are used to provide hooks into the
parent program.  Three notable instances are:

  * `PIDSource`
  * `PIDOutput`
  * `RegisterTimeFunction`

The `PIDSource` and `PIDOutput` are used for retrieving `feedback` from the
parent system and delivering the calculated `output`, respectively.  At the
construction of a `PIDController`, two function pointers are passed in which
represent functions created by the user which perform these tasks.  Below is an
example of using `PIDSource` and `PIDOutput` functions:

```cpp
int pidSource()
{
  return mySensor.getValue();
}

void pidOutput(int output)
{
  myRobot.driveAtSpeed(output);
}

// P, I, and D represent constants in the user's program
PIDController myPIDController(P, I, D, pidSource, pidOutput);
```

`RegisterTimeFunction()` is a method that gives the user a chance to tell the
`PIDController` how to retrieve system time.  This is useful because different
platforms have different APIs but we can use handling functions to pass in
references to time-getting functions (if the return type matches the
`PIDController`'s function pointer type) or to make conversions so that the
types are made to match.  Below are some examples:

* On an Arduino system, the millis() function returns time in milliseconds
as an unsigned long, which matches the function pointer type for
the registerTimeFunction() method.

```
myPIDController.registerTimeFunction(millis);
```

* On a system that has no matching time-getting function, we can create a
wrapper method that will convert the value and allow us to pass it as a
parameter.

```cpp
// Let's assume this platform's time function is
// double getSeconds();

unsigned long timeFunction()
{
  // Multiply by 1000 to convert seconds to milliseconds
  return (unsigned long) (getSeconds() * 1000);
}

myPIDController.registerTimeFunction(timeFunction);
```

## Feedback Wrap

Feedback wrap is an optional feature that can cause the number line of the input
range to "wrap" around on itself.  As an example, assume a `PIDSource` that set
to retrieve a value from a magnetometer sensor that returns a value from 0 to 360.
Under normal circumstances, a feedback of 15 and target of 345 would cause
the system to generate an `output` to navigate it in the positive direction to
compensate for the error of 330.  However, in this scenario, it would be much
more effective to generate a negative `output` to cover a distance of 30 rather
than a distance of 330.  Feedback Wrap allows us to cause the upper and lower
bounds of the range to appear adjacent to one another for the purpose of
calculating the `error`.  This also fixes problems with overshoot: if the
system's target was set to 0 and it overshot, a magnetometer or other rotation
sensor would return a value near to 360, which would cause the error to spike
quickly and make the system travel the entire distance in the negative direction
again. Setting the `feedback` to be wrapped at 0 and 360 causes the `error` to
accurately represent the physical state of the system.

To setup Feedback Wrap (this automatically enables it as well):

```cpp
myPIDController.setFeedbackWrapBounds(0, 360);
```

To disable or re-enable Feedback Wrap:

```cpp
myPIDController.setFeedbackWrapped(false);
myPIDController.setFeedbackWrapped(true);
```

## Calculation Transparency

This library provides many getter functions that will allow insight into the
system's state.  Any function that sets an option or value has a corresponding
getter which can be useful in recalling the settings that the `PIDController` is
operating under.  Additionally, methods are provided to display the contribution
of each component of PID to the current output, which is very useful when tuning
the system.

```cpp
myPIDController.getProportionalComponent();
myPIDController.getIntegralComponent();
myPIDController.getDerivativeComponent();
myPIDController.getP(); // Returns P Gain
myPIDController.getI(); // Returns I Gain
myPIDController.getD(); // Returns D Gain
myPIDController.getTarget();
myPIDController.getFeedback();
myPIDController.getOutput();
myPIDController.getError();
```

These getter functions are updated at every run of `tick()`.

## Templates

In C++, using `templates` allows you to create classes that are prepared to
handle different types of data.  This PID library uses templates to allow you
to perform calculations using different types of numeric values.  For example:

```cpp
PIDController<int> myIntPIDController(...);
PIDController<long> myLongPIDController(...);
PIDController<float> myFloatPIDController(...);
PIDController<double> myDoublePIDController(...);
```

The data type passed into the angle brackets <> determines what type of value
the PIDController will handle.  A PIDController<int> will need PIDSource
and PIDOutput function pointers that are compliant with an int, but a
PIDController<double> will need to comply with doubles.  For example:

```cpp
// For an int PIDController
int pidIntSource()
{
  return mySensor.getIntValue();
}
void pidIntOutput(int output)
{
  myRobot.setIntSpeed(output);
}
// P, I, and D represent constants in the user's program.
PIDController<int> myIntPIDController(P, I, D, pidIntSource, pidIntOutput);


// For a double PIDController
double pidDoubleSource()
{
  return mySensor.getDoubleValue();
}
void pidDoubleOutput(double output)
{
  myRobot.setDoubleSpeed(output);
}
// P, I, and D represent constants in the user's program.
PIDController<double> myDoublePIDController(P, I, D, pidDoubleSource, pidDoubleOutput);
```

Additionally, the `getter` methods covered in the "Calculation Transparency"
section and their corresponding `setter` methods will also adapt to handle the
data type specified for the PIDController at construction.
