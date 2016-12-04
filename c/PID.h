#ifndef PID_H
#define PID_H

typedef struct pid_controller {

	double p;
	double i;
	double d;
	int target;
	int output;
	uint8_t enabled;
	int currentFeedback;
	int lastFeedback;
	int error;
	int lastError;
	long currentTime;
	long lastTime;
	int integralCumulation;
	int maxCumulation;
	int cycleDerivative;

	uint8_t inputBounded;
	int inputLowerBound;
	int inputUpperBound;
	uint8_t outputBounded;
	int outputLowerBound;
	int outputUpperBound;
	uint8_t feedbackWrapped;
	int feedbackWrapLowerBound;
	int feedbackWrapUpperBound;

	uint8_t timeFunctionRegistered;
	int (*pidSource)();
	void (*pidOutput)(int output);
	unsigned long (*getSystemTime)();

} PIDController;

PIDController *createPIDController(double p, double i, double d, int (*pidSource)(void), void (*pidOutput)(int output));

void tick(PIDController *controller);
void setTarget(PIDController *controller, int t);
int getTarget(PIDController *controller);
int getOutput(PIDController *controller);
int getFeedback(PIDController *controller);
int getError(PIDController *controller);
void setEnabled(PIDController *controller, uint8_t e);
uint8_t isEnabled(PIDController *controller);
int getProportionalComponent(PIDController *controller);
int getIntegralComponent(PIDController *controller);
int getDerivativeComponent(PIDController *controller);
void setMaxIntegralCumulation(PIDController *controller, int max);
int getMaxIntegralCumulation(PIDController *controller);
int getIntegralCumulation(PIDController *controller);

void setInputBounded(PIDController *controller, uint8_t bounded);
uint8_t isInputBounded(PIDController *controller);
void setInputBounds(PIDController *controller, int lower, int upper);
int getInputLowerBound(PIDController *controller);
int getInputUpperBound(PIDController *controller);
void setOutputBounded(PIDController *controller, uint8_t bounded);
uint8_t isOutputBounded(PIDController *controller);
void setOutputBounds(PIDController *controller, int lower, int upper);
int getOutputLowerBound(PIDController *controller);
int getOutputUpperBound(PIDController *controller);
void setFeedbackWrapped(PIDController *controller, uint8_t wrap);
uint8_t isFeedbackWrapped(PIDController *controller);
void setFeedbackWrapBounds(PIDController *controller, int lower, int upper);
int getFeedbackWrapLowerBound(PIDController *controller);
int getFeedbackWrapUpperBound(PIDController *controller);

void setPID(PIDController *controller, double p, double i, double d);
void setP(PIDController *controller, double p);
void setI(PIDController *controller, double i);
void setD(PIDController *controller, double d);
double getP(PIDController *controller);
double getI(PIDController *controller);
double getD(PIDController *controller);
void setPIDSource(PIDController *controller, int (*pidSource)());
void setPIDOutput(PIDController *controller, void (*pidOutput)(int output));
void registerTimeFunction(PIDController *controller, unsigned long (*getSystemTime)(void));

#endif // PID_H
