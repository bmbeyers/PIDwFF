/******************************************************************************
 * Arduino PID with Feed-Forward Controller Library                           *
 * by Brandon Beyers <bmbeyers@gmail.com>                                     *
 * This library is based on PID library, with original source by Brett        *
 * Beauregard <br3ttb@gmail.com> brettbeauregard.com                          *
 *                                                                            *
 * This library is licensed under MIT license.                                *
 *                                                                            *
 * Refer to header file for additional information on PID controller.         *
 ******************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PIDwFF.h>

/* Public functions ***********************************************************/
PID::PID(double* Setpoint, double* Input, double* Output, bool Inverse) {
  _setpoint = Setpoint;
  _input = Input;
  _output = Output;
  PID::setInverse(Inverse);
}

PID::PID(double* Setpoint, double* Input, double* Output)
  :PID::PID(Setpoint, Input, Output, false) {}

void PID::setProportional(double Kp) { PID::setTuning(_userKp, Kp); }

void PID::setIntegral(double Ki) { PID::setTuning(_userKi, Ki); }

void PID::setDerivative(double Kd) { PID::setTuning(_userKd, Kd); }

void PID::setFeedForward(double Kf) { PID::setTuning(_userKf, Kf); }

void PID::setSampleTime(int NewSampleTime) {
  if ( NewSampleTime < MIN_SAMPLE_TIME || _enabled ) return;
  _sampleTime = (unsigned long)NewSampleTime;
}

void PID::setOutputLimits(double min, double max) {
  if ( max >= min ) {
    _minOutputLimit = min;
    _maxOutputLimit = max;
  } else {  // handle scenario where limits are provided backwards
    _minOutputLimit = max;
    _maxOutputLimit = min;
  }
  _outputLimitEnabled = true;
}

void PID::setNoOutputLimits() { _outputLimitEnabled = false; }

void PID::setIntegratorLimits(double min, double max) {
  if ( max >= min ) {
    _minIntegratorLimit = min;
    _maxIntegratorLimit = max;
  } else {  // handle scenario where limits are provided backwards
    _minIntegratorLimit = max;
    _maxIntegratorLimit = min;
  }
  _integratorLimitEnabled = true;
}

void PID::setNoIntegratorLimits() { _integratorLimitEnabled = false; }

void PID::setInverse(bool Inverse) { _inverted = Inverse; }

void PID::enableController() {
  if ( !PID::validTuning() ) return;
  if ( !_enabled ) PID::initializeController();
  _enabled = true;
}

void PID::disableController() { _enabled = false; }

bool PID::compute() {
  /* This is, as they say, where the magic happens. This function should be
   * called every time "void loop()" executes. The function will decide for
   * itself whether a new PID output needs to be computed, and will return true
   * when it has done so; otherwise, it will return false. */
  if ( !_enabled ) return false;

  unsigned long timeNow = millis();
  unsigned long timeChange = (timeNow - _lastTime);
  if ( timeChange >= _sampleTime ) {
    _lastTime = timeNow;
    double input = *_input;
    double error = *_setpoint - input;
    double deltaError = (error - _lastError);
    _lastError = error;

    _KpOut = _Kp * error;

    _KiOut += _Ki * error;
    if ( _integratorLimitEnabled ) {
      if ( _KiOut > _maxIntegratorLimit ) _KiOut = _maxIntegratorLimit;
      if ( _KiOut < _minIntegratorLimit )  _KiOut = _minIntegratorLimit;
    }

    _KdOut = _Kd * deltaError;

    _KfOut = _Kf * input;

    double Output = _KpOut + _KiOut + _KdOut + _KfOut;

    if ( _inverted ) Output = -Output;
    if ( _outputLimitEnabled ) {
      if ( Output > _maxOutputLimit ) Output = _maxOutputLimit;
      if ( Output < _minOutputLimit ) Output = _minOutputLimit;
    }

	  *_output = Output;
	  return true;
  } else return false;
}

double PID::getKp() { return _userKp; }
double PID::getKi() { return _userKi; }
double PID::getKd() { return _userKd; }
double PID::getKf() { return _userKf; }

double PID::getKpOutput() { return _KpOut; }
double PID::getKiOutput() { return _KiOut; }
double PID::getKdOutput() { return _KdOut; }
double PID::getKfOutput() { return _KfOut; }

bool PID::getEnabledStatus() { return _enabled; }
bool PID::getInverseStatus() { return _inverted; }

bool PID::getOutputLimitStatus() { return _outputLimitEnabled; }
double PID::getOutputMinLimit() { return _minOutputLimit; }
double PID::getOutputMaxLimit() { return _maxOutputLimit; }

bool PID::getIntegratorLimitStatus() { return _integratorLimitEnabled; }
double PID::getIntegratorMinLimit() { return _minIntegratorLimit; }
double PID::getIntegratorMaxLimit() { return _maxIntegratorLimit; }

int PID::getSampleTime() { return _sampleTime; }
double PID::getSampleTimeInSeconds() {
  return (double)PID::getSampleTime() / 1000.0; }

/* Private functions **********************************************************/
void PID::setTuning(double &userGain, double value) {
  if ( value < MIN_GAIN ) return;
  userGain = value;
  if ( _enabled ) setControllerTuning();
}

void PID::initializeController() {
  PID::setControllerTuning();
  /* Do as much as you can to ensure a bumpless transfer: */
  _KiOut = *_output;
  _lastError = *_setpoint - *_input;
  _lastTime = (unsigned long)millis();
}

bool PID::validTuning() {
  /* Cannot have any gains be negative: */
  if ( _userKp < 0.0 || _userKi < 0.0 || _userKd < 0.0 || _userKf < 0.0 ) {
    return false;
  }
  /* Need at least Kp or Ki to be non-zero: */
  if ( _userKp > 0.0 || _userKi > 0.0 ) { return true; } else { return false; }
}

void PID::setControllerTuning() {
  if ( !PID::validTuning() ) { PID::disableController(); return; }
  _Kp = _userKp;
  _Ki = _userKi * PID::getSampleTimeInSeconds();
  _Kd = _userKd / PID::getSampleTimeInSeconds();
  _Kf = _userKf;
}
