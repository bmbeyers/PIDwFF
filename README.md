# PIDwFF
PID controller with optional feed-forward gain

Arduino PID with Feed-Forward Controller Library
by Brandon M Beyers <bmbeyers@gmail.com>
original source Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

This library is licensed under MIT license.

The common PID Controller is implemented in a closed feedback loop and has a maximum of three terms: the proportional (P) term, the integral (I) term, and the derivative (D) term. It is also common to have controllers with one or two of these terms set to zero, resulting in a simpler type controller, such as P, I, PI, or PD controllers. The controller acts on the measured error between the Input and Setpoint parameter values. Improperly tuned PID controllers can demonstrate unstable behavior, which could lead to equipment damage. For this
reason, use of this controller should be used only by competent individuals.

The additional feed-forward control is an open-loop gain based on the Setpoint parameter value, which may be used to improve the transient response of the controller. This optional feature requires fundamental knowledge of the steady-state relationship between Setpoint and Output, which may not be consistent across all modes of operation. For this reason, use of this controller should be used only by competent individuals.

The effective block diagram of this controller is shown below:

```
                                     +----+
             +---------------------->| Kf |-----+
             |                       +----+     |
             |                                  |
             |                            _Imax |
             |                           /      |
             |                       +----+     |
             |                       | Ki |     |   +---+
             |                 +---->| -- |--+  |   |   |
             |                 |     |  s |  |  +-->| + |
             |   +---+         |     +----+  |      |   |         _Omax
             |   |   |         | Imin_/      +----->| + |        /
  Setpoint --+-->| + |         |     +----+         |   |------------>Output
                 |   |--error--+---->| Kp |-------->| + | Omin_/
     Input ----->| - |         |     +----+         |   |
                 |   |         |                +-->| + |
                 +---+         |   +-------+    |   |   |
                               |   |  sKd  |    |   +---+
                               +-->| ----- |----+
                                   | 1+sTd |
                                   +--------
```

Notes:

- Derivative block time constant (Td) is set implicitly based on the chosen SampleTime setting.

- This PID class library includes a simplified constructor, as compared to the original PID library, requiring only the Setpoint, Input, and Output pointer references. An optional Inverse boolean is also allowed following these required parameters.

- This PID class library also includes an optional feed-forward gain to improve the dynamic response of the system. By default, it is set at zero, so the class behaves as a standard PID controller without any explicit call from the user.

- The Proportional, Integral, Derivative, and FeedForward gains are all initialized at zero, and must be set individually using dedicated methods. This allows for several variations of controllers available; however, the controller cannot be enabled unless either Kp or Ki are used (>0). Examples (without feed-forward gain) include P, I, PI, PD, PID, and ID.
