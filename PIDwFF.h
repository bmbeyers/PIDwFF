/******************************************************************************
 * Arduino PID with Feed-Forward Controller Library                           *
 * by Brandon M Beyers <bmbeyers@gmail.com>                                   *
 * original source Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com    *
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
 * Setpoint parameter value, which may be used to improve the transient       *
 * response of the controller. This optional feature requires fundamental     *
 * knowledge of the steady-state relationship between Setpoint and Output,    *
 * which may not be consistent across all modes of operation. For this reason,*
 * use of this controller should be used only by competent individuals.       *
 *                                                                            *
 * The effective block diagram of this controller is shown below:             *
 *                                                                            *
 *                                                                            *
 *                                    +----+ Of                               *
 *            +---------------------->| Kf |-----+                            *
 *            |                       +----+     |                            *
 *            |                                  |                            *
 *            |                            _Imax |                            *
 *            |                           /      |                            *
 *            |                       +----+     |                            *
 *            |                       | Ki | Oi  |   +---+                    *
 *            |                 +---->| -- |--+  |   |   |                    *
 *            |                 |     |  s |  |  +-->| + |                    *
 *            |   +---+         |     +----+  |      |   |         _Omax      *
 *            |   |   |         | Imin_/      +----->| + |        /           *
 * Setpoint --+-->| + |         |     +----+ Op      |   |------------>Output *
 *                |   |--error--+---->| Kp |-------->| + | Omin_/             *
 *    Input ----->| - |         |     +----+         |   |                    *
 *                |   |         |                +-->| + |                    *
 *                +---+         |   +-------+    |   |   |                    *
 *                              |   |  sKd  | Od |   +---+                    *
 *                              +-->| ----- |----+                            *
 *                                  | 1+sTd |                                 *
 *                                  +--------                                 *
 *                                                                            *
 *                                                                            *
 * NOTE: Derivative block time constant (Td) is set implicitly based on the   *
 * chosen SampleTime setting.                                                 *
 ******************************************************************************/

#ifndef PIDwFF_h
#define PIDwFF_h

class PID {
  public:
    /* Constructor: Links the PID controller with the Input, Output, and
     * Setpoint pointers; allows the user to invert the output. PID and feed-
     * forward gains are all initialized at zero, and must be set manually
     * before the controller can be enabled. */
    PID(double* Setpoint, double* Input, double* Output, bool Inverse);

    /* Overloaded constructor: Inverse automatically set to false. */
    PID(double* Setpoint, double* Input, double* Output);

    /* Set the PID and feed-forward tuning values: */
    void setProportional(double Kp);
    void setIntegral(double Ki);
    void setDerivative(double Kd);
    void setFeedForward(double Kf);

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

    /* Display functions: These functions query the PID controller for the
     * internally-used values. It is intended for front-end use, where the user
     * needs to know what values are actually being used inside the control. */
  	double getKp();
  	double getKi();
  	double getKd();
    double getKf();

    double getKpOutput();
    double getKiOutput();
    double getKdOutput();
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

  private:
    /* Generic function to set tuning gain: */
    void setTuning(double &userGain, double value);

    /* Initialize the controller using the user-provided gains and settings: */
	  void initializeController();

    /* Validation check of the user-provided gains: */
    bool validTuning();

    /* Convert user PID settings into those used by the contorller: */
    void setControllerTuning();

    /* Minimum tuning gains and time constant settings: */
    const double MIN_GAIN = 0.0;
    const int MIN_SAMPLE_TIME = 5;

    /* User-provided tuning gain values: */
	  double _userKp = MIN_GAIN;
	  double _userKi = MIN_GAIN;
	  double _userKd = MIN_GAIN;
    double _userKf = MIN_GAIN;

    /* Calculated tuning gain values set during the initialization process: */
	  double _Kp;
    double _Ki;
    double _Kd;
    double _Kf;

    /* Stored PID outputs: */
    double _KpOut;
    double _KiOut;
    double _KdOut;
    double _KfOut;

    /* Stored derivative integrator, required for derivative gain: */
    double _lastError;

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
