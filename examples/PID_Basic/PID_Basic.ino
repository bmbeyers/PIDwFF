/******************************************************************************
 * PID_Basic Example                                                          *
 * by Brandon M Beyers <bmbeyers@gmail.com>                                   *
 *                                                                            *
 * This example code allows you to test the PID controller on a ficticious    *
 * plant system. The plant is defined as either a first-order filter or lead- *
 * lag filter (compensator block used). NOTE: This requires the use of the    *
 * LeadLag library in order to function properly. If this library is not      *
 * desired, code may be commented / uncommented out to make this example work *
 * using just a first-order filter for the plant.                             *
 *                                                                            *
 * Refer to the library documentation for instructions on the PID controller. *
 *                                                                            *
 * This example reads in an analog input to be used as the setpoint of the    *
 * PID controller. The analog input should be connected to a potentiometer    *
 * providing a value between 0 and IOREF (5V or 3.3V, depending on board).    *
 * The 'output' of this code is purely simulated, and does not apply any      *
 * voltages to any of the digital IO pins of the Arduino. To view the output, *
 * use the Arduino IDE's Serial Plotter. The plotter will show the setpoint   *
 * provided by the potentiometer / analog input pin, as well as the actual    *
 * output value of the plant.                                                 *
 ******************************************************************************/

#include <PIDwFF.h>
#include <LeadLag.h>  // comment out if library not available

#define PIN_INPUT 0

// Define Variables we'll be connecting the PID controller to:
double Setpoint, Actual, ControlSignal;

// Define the tuning gains:
double Kp = 100.0;
double Ki = 50.0;
double Kd = 20.0;
double Kf = 1.0;

// Define the plant lead-lag time constants:
double PlantLeadTC = 3.0;  // this value won't be used if no LeadLag library
double PlantLagTC = 8.0;

// Contstruct PID controller and specify the links to variables:
PID pid(&Setpoint, &Actual, &ControlSignal);

// Construct the Plant representation using LeadLag object, and specify links
// to variables:
LeadLag plant(&ControlSignal, &Actual);  // comment out if no LeadLag library

const int SAMPLE_TIME_MS = 10;

const int MIN_SETPOINT = -100;
const int MAX_SETPOINT = 100;
const int MID_SETPOINT = (MIN_SETPOINT + MAX_SETPOINT) / 2.0;

const double PID_CEILING_FACTOR = 5.0;
const double MIN_PID_OUT = (double)MIN_SETPOINT * PID_CEILING_FACTOR;
const double MAX_PID_OUT = (double)MAX_SETPOINT * PID_CEILING_FACTOR;

void setup() {
  // Initialize the Serial communications:
  Serial.begin(115200);
  while ( !Serial ) ;
  
  // Initialize the PID controller:
  pid.setProportional(Kp);
  pid.setIntegral(Ki);
  pid.setDerivative(Kd);
  pid.setFeedForward(Kf);
  pid.setSampleTime(SAMPLE_TIME_MS);
  pid.setOutputLimits(MIN_PID_OUT, MAX_PID_OUT);

  // initialize the plant -- comment this section out if no LeadLag:
  plant.setLeadTC(PlantLeadTC);
  plant.setLagTC(PlantLagTC);
  plant.setSampleTime(SAMPLE_TIME_MS);

  // Initialize the plant output value at the middle of the setpoint range:
  Actual = MID_SETPOINT;
  ControlSignal = MID_SETPOINT;

  Serial.println("");
  Serial.println("PID Controller:");
  Serial.print("Kp = ");
  Serial.println(pid.getKp());
  Serial.print("Ki = ");
  Serial.println(pid.getKi());
  Serial.print("Kd = ");
  Serial.println(pid.getKd());
  Serial.print("Kf = ");
  Serial.println(pid.getKf());

  Serial.println("");
  Serial.println("Plant Transfer Function:");
  Serial.print("Lead TC = ");
  Serial.println(plant.getLeadTC());
  Serial.print("Lag TC = ");
  Serial.println(plant.getLagTC());

  delay(2000);

  // Turn the PID controller on:
  pid.enableController();
  plant.enableCompensator();
}

void loop() {
  int value = analogRead(PIN_INPUT);
  Setpoint = (double)map(value, 0, 1023, MIN_SETPOINT, MAX_SETPOINT);
  if ( pid.compute() ) {
    // comment the following line out if no LeadLag, and uncomment the line
    // below it to use instead:
    while ( !plant.compute() ) { ; }
//    Actual += (Actual - ControlSignal) / PlantLagTC * pid.getSampleTimeInSeconds();
    Serial.print(Setpoint);
    Serial.print(',');
    Serial.println(Actual);
  }
}


