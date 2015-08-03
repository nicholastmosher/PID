# PID
A simple PID library for use with Arduino or other C++ applications.

A PID Controller is a method of system control in which a correctional output
is generated to guide the system toward a desired setpoint (aka target).
The PID Controller calculates the output based on the following factors:
 
 * Gains (proportional, integral, and derivative)
 * Target
 * Feedback

The gain values act as multipliers for their corresponding components of PID
(more detail later).  The target is the value which the system strives to
reach by manipulating the output.  The feedback is the system's actual
position or status in regards to the physical world.
Another important term in PID is "error", which refers to the difference
between the target and the feedback.

Each of the three components of PID contributes a unique behavior to the
system.

##Proportional

The Proportional component introduces a linear relationship between the
error (target minus feedback) and the output.  This means that as the
feedback grows further away from the target, the output grows
proportionally stronger.

    Proportional component = (P Gain) * (target - feedback)

##Integral

The Integral component is designed to give a very precise approach of the
feedback to the target.  Depending on the scale of the physical system
and the precision of feedback (e.g. sensors), the proportional component
alone is likely not sufficient to provide adequate power (e.g. to motors)
to guide the system in regards to small-scale correction.  The Integral
component integrates the error of the system (target - feedback) over
time.  If the system reaches a point where it is close but not exactly
on top of the target, the integration will slowly build until it is
powerful enough to overcome static resistances and move the system
preciesly to the target.

    Integral component = (I Gain) * Integral of error over time

      *In this implementation, Integral is calculated with a running
      summation of the system's error, updated at each tick.

##Derivative

The Derivative component measures the rate of change of the feedback.
It can reduce the strength of the output if the feedback is approaching
the target too quickly or if the feedback is moving away from the target.

    Derivative component = (D Gain) * ((error - lastError) / (time - lastTime))
