/******************************************************************************
 * Arduino Enhanced PID Controller Library                                    *
 * by Brandon Beyers <bmbeyers@gmail.com>                                     *
 *                                                                            *
 * This library is based on PID library, with original source by Brett        *
 * Beauregard <br3ttb@gmail.com> brettbeauregard.com                          *
 *                                                                            *
 * This library is licensed under MIT license.                                *
 *                                                                            *
 *                                                                            *
 * The common PID Controller is implemented in a closed feedback loop and has *
 * a maximum of three terms: the proportional (P) term, the integral (I)      *
 * term, and the derivative (D) term. It is also common to have controllers   *
 * with one or two of these terms set to zero, resulting in a simpler type    *
 * controller, such as P, I, PI, or PD controllers. The controller acts on    *
 * the measured error between Input parameter value and Setpoint parameter    *
 * value. Improperly tuned PID controllers can demonstrate unstable behavior, *
 * which could lead to equipment damage. For this reason, use of this         *
 * controller should be used only by competent individuals.                   *
 *                                                                            *
 * The additional feed-forward control is an open-loop gain based on the      *
 * Setpoint parameter value, which, together with the lead-lag compensator,   *
 * may be used to improve the transient response of the controller. This      *
 * optional feature requires fundamental knowledge of the steady-state        *
 * relationship between Setpoint and Output, which may not be consistent      *
 * across all modes of operation. For this reason, use of this controller     *
 * should be used only by competent individuals.                              *
 *                                                                            *
 * In addition to the feed-forward control, the feedback proportional gain,   *
 * Kv, converts a standard PID controller into a velocity form PID controller.*
 * The velocity form PID control may be used to produce stable behavior and   *
 * minimize overshoot of integrating processes. The benefit of the separate   *
 * control gain lies in the ability to retain the error proportional gain and *
 * obtain a faster rise time.                                                 *
 *                                                                            *
 * The effective block diagram of this controller is shown in the README file *
 ******************************************************************************/

#ifndef PID_Plus_h
#define PID_Plus_h

class PID {
  public:
    /* Constructor: Links the PID controller with the Input, Output, and
     * Setpoint pointers; allows the user to invert the output. PID and feed-
     * forward gains are all initialized at zero, and must be set manually
     * before the controller can be enabled. */
    PID(double* Setpoint, double* Input, double* Output, bool Inverse);

    /* Overloaded constructor: Inverse automatically set to false. */
    PID(double* Setpoint, double* Input, double* Output);

    /* (Re)set the Input, Output, or Setpoint pointers: */
    void changeSetpoint(double* NewSetpoint);
    void changeInput(double* NewInput);
    void changeOutput(double* NewOutput);

    /* Set the PID and feed-forward tuning values: */
    void setProportional(double Kp);
    void setIntegral(double Ki);
    void setDerivative(double Kd);
    void SetVelocity(double Kv);
    void setFeedForward(double Kf);

    void setDerivativeTime(int Td);
    void setLeadTime(int Tc);
    void setLagTime(double Tb);

    void setBiasValue(double b);

    /* Set the sample time:
     * NOTE: Only allowed when controller is not enabled. */
    void setSampleTime(int NewSampleTime);

    /* Set the minimum and maximum limits for Output and Integrator: */
    void setOutputLimits(double min, double max);
    void setNoOutputLimits();
    void setIntegratorLimits(double min, double max);
    void setNoIntegratorLimits();

    /* Invert the calculated output: */
    void setInverse(bool Inverse);

    /* Enables the controller to compute Output; initializes when enabled: */
    void enableController();
    void disableController();

    /* Performs the PID calculation; returns true if output is computed:
     * NOTE: This should be called every time loop() cycles. */
    bool compute();

    double getSetpoint();
    double getInput();
    double getOutput();

    /* Display functions: These functions query the PID controller for the
     * internally-used values. It is intended for front-end use, where the user
     * needs to know what values are actually being used inside the control. */
    double getKp();
  	double getKi();
  	double getKd();
    double getKv();
    double getKf();

    double getBias();

    double getKpOutput();
    double getKiOutput();
    double getKdOutput();
    double getKvOutput();
    double getKfOutput();

  	bool getEnabledStatus();
	  bool getInverseStatus();

    bool getOutputLimitStatus();
    double getOutputMinLimit();
    double getOutputMaxLimit();

    bool getIntegratorLimitStatus();
    double getIntegratorMinLimit();
    double getIntegratorMaxLimit();

    int getSampleTime();
    double getSampleTimeInSeconds();

    int getDerivativeTime();
    double getDerivativeTimeInSeconds();

    int getLeadTime();
    double getLeadTimeInSeconds();

    int getLagTime();
    double getLagTimeInSeconds();

    bool getLeadLagInService();

  private:
    /* Generic functions to set tuning gains or filter time constants: */
    void setTuningGain(double &userGain, double value);
    void setFilterTimeConstant(int &userTime, int value);

    /* Initialize the controller: */
	  void initializeController();

    /* Validation check of the controller gains: */
    bool validTuning();

    /* Functions to call for private variables: */
    double getLastSetpoint();
    double getLastInput();

    double getError();
    double getDeltaSetpoint();
    double getDeltaInput();

    void updateKpOutput();
    void updateKiOutput();
    void updateKdOutput();
    void updateKvOutput();
    void updateKfOutput();

    void updateOutput();

    /* Minimum tuning gains and time constant settings: */
    const double MIN_GAIN = 0.0;
    const int MIN_SAMPLE_TIME = 5;
    const int MIN_FILTER_TIME = 0;

    /* Controller tuning gain  and filter time constant values: */
	  double _Kp = MIN_GAIN;
    double _Ki = MIN_GAIN;
    double _Kd = MIN_GAIN;
    double _Kv = MIN_GAIN;
    double _Kf = MIN_GAIN;

    int _Td = MIN_FILTER_TIME;
    int _Tc = MIN_FILTER_TIME;
    int _Tb = MIN_FILTER_TIME;

    double _bias = MIN_GAIN;

    /* Stored PID outputs: */
    double _KpOut;
    double _KiOut;
    double _KdOut;
    double _KvOut;
    double _KfOut;

    /* Stored previous values for derivative and lead-lag compensator: */
    double _lastSetpoint;
    double _lastInput;

    /* Output and Integrator limit settings: */
    double _minOutputLimit = 0.0;
    double _maxOutputLimit = 255.0;
    double _minIntegratorLimit = 0.0;
    double _maxIntegratorLimit = 255.0;

    /* Boolean settings: */
	  bool _inverted;
	  bool _enabled = false;
    bool _outputLimitEnabled = false;
    bool _integratorLimitEnabled = false;

    /* Pointers to the Input, Output, and Setpoint variables:
     * This creates a hard link between the variables and the PID controller,
     * freeing the user from having to constantly tell us what these values are.
     * With pointers, the controller just accesses it automatically. */
    double *_input;
    double *_output;
    double *_setpoint;

    /* Timing Variables: */
    unsigned long _sampleTime;
	  unsigned long _lastTime;
};
#endif
