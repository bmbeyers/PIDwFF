/******************************************************************************
 * Arduino Enhanced PID Controller Library                                    *
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

#include <PID_Plus.h>

/* Public functions ***********************************************************/
// Constructor functions:
PID::PID(double* Setpoint, double* Input, double* Output, bool Inverse) {
  PID::changeSetpoint(Setpoint);
  PID::changeInput(Input);
  PID::changeOutput(Output);
  PID::setInverse(Inverse);
}

PID::PID(double* Setpoint, double* Input, double* Output)
  :PID::PID(Setpoint, Input, Output, false) {}

// (Re)set the Input, Output, or Setpoint pointers:
void PID::changeSetpoint(double* newSetpoint) { _setpoint = newSetpoint; }
void PID::changeInput(double* newInput) { _input = newInput; }
void PID::changeOutput(double* newOutput) { _output = newOutput; }

// Set the controller tuning gains:
void PID::setProportional(double Kp) {
  // PID controller requires that Kp or Ki (or both) > 0 to be enabled, so we
  // need to check for this before setting the control.
  if ( PID::getEnabledStatus() && PID::getKi() == MIN_GAIN ) {
    // Controller is enabled and Ki is already set to 0, so don't allow Kp = 0:
    if ( Kp == MIN_GAIN ) return;
  }
  // otherwise:
  PID::setTuningGain(_Kp, Kp);
}

void PID::setIntegral(double Ki) {
  // PID controller requires that Kp or Ki (or both) > 0 to be enabled, so we
  // need to check for this before setting the control.
  if ( PID::getEnabledStatus() && PID::getKp() == MIN_GAIN ) {
    // Controller is enabled and Kp is already set to 0, so don't allow Ki = 0:
    if ( Ki == MIN_GAIN ) return;
  }
  // otherwise:
  PID::setTuningGain(_Ki, Ki);
}

void PID::setDerivative(double Kd) { PID::setTuningGain(_Kd, Kd); }
void PID::SetVelocity(double Kv) { PID::setTuningGain(_Kv, Kv); }
void PID::setFeedForward(double Kf) { PID::setTuningGain(_Kf, Kf); }

// Set the controller filter time constants:
void PID::setDerivativeTime(int Td) { PID::setFilterTimeConstant(_Td, Td); }
void PID::setLeadTime(int Tc) { PID::setFilterTimeConstant(_Tc, Tc); }
void PID::setLagTime(int Tb) { PID::setFilterTimeConstant(_Tb, Tb); }

// Bias value is not restricted to non-negative values in the same way that the
// other controller gains are
void PID::setBiasValue(double b) { _bias = b; }

void PID::setSampleTime(int NewSampleTime) {
  if ( PID::getEnabledStatus() ) return;  // Cannot change this while active
  if ( NewSampleTime < MIN_SAMPLE_TIME ) NewSampleTime = MIN_SAMPLE_TIME;
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
  if ( !PID::getEnabledStatus() ) return false;

  unsigned long timeNow = millis();
  unsigned long timeChange = (timeNow - _lastTime);
  if ( timeChange >= _sampleTime ) {
    _lastTime = timeNow;  // change last time to current time for next iteration

    // Calculate the individual output values:
    PID::updateKpOutput();
    PID::updateKiOutput();
    PID::updateKdOutput();
    PID::updateKvOutput();
    PID::updateKfOutput();

    // Update the final output value:
    PID::updateOutput();

    // Update the (now) previous values:
    _lastSetpoint = PID::getSetpoint();
    _lastInput = PID::getInput();

	  return true;
  } else return false;
}

double PID::getSetpoint() { return *_setpoint; }
double PID::getInput() { return *_input; }
double PID::getOutput() { return *_output; }

double PID::getKp() { return _Kp; }
double PID::getKi() { return _Ki; }
double PID::getKd() { return _Kd; }
double PID::getKv() { return _Kv; }
double PID::getKf() { return _Kf; }

double PID::getBias() { return _bias; }

double PID::getKpOutput() { return _KpOut; }
double PID::getKiOutput() { return _KiOut; }
double PID::getKdOutput() { return _KdOut; }
double PID::getKvOutput() { return _KvOut; }
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

int PID::getDerivativeTime() { return _Td; }
double PID::getDerivativeTimeInSeconds() {
  return (double)PID::getDerivativeTime() / 1000.0; }

int PID::getLeadTime() { return _Tc; }
double PID::getLeadTimeInSeconds() {
  return (double)PID::getLeadTime() / 1000.0; }

int PID::getLagTime() { return _Tb; }
double PID::getLagTimeInSeconds() {
  return (double)PID::getLagTime() / 1000.0; }

bool PID::getLeadLagInService() { return (PID::getLagTime() > MIN_FILTER_TIME); }

/* Private functions **********************************************************/
void PID::setTuningGain(double &userGain, double value) {
  if ( value < MIN_GAIN ) value = MIN_GAIN;  // restrict minimum value
  userGain = value;
}

void PID::setFilterTimeConstant(int &userTime, int value) {
  if ( value < MIN_FILTER_TIME ) value = MIN_FILTER_TIME;  // restrict minimum value
  userTime = value;
}

void PID::initializeController() {
  if ( PID::getKi() > MIN_GAIN ) {
    // Integral gain is used, so steady-state error should be zero.
    // In steady state, error and derivative are both 0, but the feed-forward
    // and velocity terms will have an effect on the steady-state. Since error
    // is 0, setpoint and input must be identical. We will use Setpoint and
    // assume that Input is equal:
    _KiOut = PID::getOutput() - PID::getBias() - ( PID::getKf() - PID::getKv() ) * PID::getSetpoint();
  }
  // Set the previous values, which will be used the first time compute() is run:
  _lastSetpoint = PID::getSetpoint();
  _lastInput = PID::getInput();
  _lastTime = (unsigned long)millis();
}

bool PID::validTuning() {
  /* Cannot have any gains be negative: */
  if ( _Kp < 0.0 || _Ki < 0.0 || _Kd < 0.0 || _Kv < 0.0 || _Kf < 0.0 ) {
    return false;
  }
  /* Need at least Kp or Ki to be non-zero: */
  return ( _Kp > 0.0 || _Ki > 0.0 );
}

/* Functions to call for private variables */
double PID::getLastSetpoint() { return _lastSetpoint; }
double PID::getLastInput() { return _lastInput; }

double PID::getError() { return PID::getSetpoint() - PID::getInput(); }
double PID::getDeltaSetpoint() { return PID::getSetpoint() - PID::getLastSetpoint(); }
double PID::getDeltaInput() { return PID::getInput() - PID::getLastInput(); }

void PID::updateKpOutput() { _KpOut = PID::getKp() * PID::getError(); }
void PID::updateKiOutput() {
  _KiOut += PID::getKi() * PID::getError() * PID::getSampleTimeInSeconds();
  if ( _integratorLimitEnabled ) {
    if ( _KiOut > _maxIntegratorLimit ) _KiOut = _maxIntegratorLimit;
    if ( _KiOut < _minIntegratorLimit )  _KiOut = _minIntegratorLimit;
  }
}
void PID::updateKdOutput() {
  double TTd = PID::getSampleTimeInSeconds() + PID::getDerivativeTimeInSeconds();
  double Dout = PID::getKdOutput() * PID::getDerivativeTimeInSeconds() + PID::getKd() * PID::getDeltaInput();
  _KdOut = Dout / TTd;
}
void PID::updateKvOutput() { _KvOut = PID::getKv() * PID::getInput(); }
void PID::updateKfOutput() {
  if ( PID::getLeadLagInService() ) {
    double TTc = PID::getSampleTimeInSeconds() + PID::getLeadTimeInSeconds();
    double TTb = PID::getSampleTimeInSeconds() + PID::getLagTimeInSeconds();
    double LLout = PID::getKfOutput() / PID::getKf() * PID::getLagTimeInSeconds();
    LLout += TTc * PID::getSetpoint() - PID::getLeadTimeInSeconds() * PID::getLastSetpoint();
    _KfOut = LLout * PID::getKf() / TTb;
  } else { _KfOut = PID::getKf() * PID::getSetpoint(); }
}
void PID::updateOutput() {
  double Output = PID::getKpOutput() + PID::getKiOutput() - PID::getKdOutput() + PID::getKvOutput() + PID::getKfOutput() + PID::getBias();

  if ( _outputLimitEnabled ) {
    if ( Output > _maxOutputLimit ) Output = _maxOutputLimit;
    if ( Output < _minOutputLimit ) Output = _minOutputLimit;
  }
  if ( _inverted ) Output = -Output;

  // Set the output value:
  *_output = Output;
}
