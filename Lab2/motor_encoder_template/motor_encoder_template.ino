/**********************************************************************************

  Arduino code to test a DC motor and encoder
  MIT Department of Mechanical Engineering, 2.004
  Rev. 1.0 -- 08/25/2020 H.C.
  Rev. 2.0 -- 03/17/2021 H.C.
  Rev. 3.0 -- 09/22/2021 H.C.

***********************************************************************************/

// set the desired pwm value (an integer between -255 and 255)
// the motor may need a threshold pwm (deadband) to actuate
# define desired_pwm 150

//# define POT   // use pot to adjust speed

# define vc2pwm 51.0    // pwm = 255 corresponds to 5v from pwm pin (255/5)

#define period_us 10000  // microseconds (1 sec = 1000000 us)

// ================================================================
// ===               SERIAL OUTPUT CONTROL                      ===
// ================================================================

// Switch between built-in Serial Plotter and Matlab streaming
// comment out both to disable serial output

//#define PRINT_DATA
#define MATLAB_SERIAL_READ

// ================================================================
// ===               DFROBOT MOTOR SHIELD DEFINITION            ===
// ================================================================

// DRI0017 motor shield
const int
PWM_A   = 10,
DIR_A   = 12;

int Pot_Pin = A5;    // select the input pin for the potentiometer

// ================================================================
// ===               ENCODER DEFINITION                         ===
// ================================================================

// The motor has a dual channel encoder and each has 12 counts per revolution (CPR)
// The effective resolution is 48 CPR with quadrature decoding. With gear reduction
// of 34, the motor output shaft's encoder resolution becomes 34*48 = 1632 CPR
// Manually calibrate the conversion factor if necessary.

float C2Rad = 34 * 12 * 4 / (2 * PI); // conversion factor for nominal encoder counts to radians

// Encoder outputs to interrupt pins with channel A to D2 (INT0) and B to D3 (INT1)
const byte encoder0pinA = 2;
const byte encoder0pinB = 3;

long   count = 0;
volatile long counter = 0;    // Inrerrupt counter (position)

// ================================================================
// ===               VARIABLE DEFINITION                        ===
// ================================================================

float wheel_pos = 0.0, wheel_vel = 0.0, pre_wheel_pos = 0.0, filt_vel = 0.0;
float alpha = 0.4851936;    // low-pass filter constant
unsigned long timer = 0;
double loop_time = 0.01;
int pwm = 0;
float vc;   // volts

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Serial.begin(115200);

  // configure motor shield M1 outputs
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);

  // initialize encoder
  pinMode(encoder0pinA, INPUT);
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0pinB), ISR_B, CHANGE);

  delay(10);    // delay 0.01 second
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (micros() - timer >= period_us) {
    timer = micros();
    count = counter;

    // wheel position and velocity from the encoder counts
    wheel_pos = count / C2Rad;    // convert to radians
    wheel_vel = (wheel_pos-pre_wheel_pos)/loop_time;
    filt_vel = alpha*wheel_vel + (1-alpha)*filt_vel;
    pre_wheel_pos = wheel_pos;

    // ================================================================
    // ===                    MOTOR PWM CODE                        ===
    // ================================================================

#ifdef POT
    pwm = map(analogRead(Pot_Pin), 0, 1023, -255, 255);
#else
    pwm = desired_pwm;
#endif

    vc = pwm / vc2pwm;

    // set the direction of rotation: LOW - forward; HIGH - backward
    if (pwm > 0) {
      digitalWrite(DIR_A, LOW);
    }
    else {
      digitalWrite(DIR_A, HIGH);
    }

    // constrain pwm to +/- 255
    pwm = constrain(pwm, -255, 255);

    // write the pwm to set angular velocity
    analogWrite(PWM_A, abs(pwm));

#ifdef PRINT_DATA
//    Serial.print(count);  Serial.print("\t");
//    Serial.print(wheel_pos); Serial.print("\t");
    Serial.print(wheel_vel);    Serial.print("\t");
//    Serial.print(filt_vel); Serial.print("\t");
//    Serial.print(apwm;  Serial.print("\t");
//    Serial.print(vc);  Serial.print("\t");
//    Serial.print(loop_time, 6); Serial.print("\t");
    Serial.print("\n");
#endif

#ifdef MATLAB_SERIAL_READ
    Serial.print(timer / 1000000.0);   Serial.print("\t");
    Serial.print(wheel_vel);    Serial.print("\t");
    Serial.print(filt_vel);    Serial.print("\t");
    Serial.print(vc);
    Serial.print("\n");
#endif

    //  delay(8);
  }
  loop_time = (micros() - timer) / 1000000.0;  //compute actual sample time
}

// ================================================================
// ===                   Interrupt Service Routines             ===
// ================================================================

// Interrupt Service Routine for encoder A (pin 2)
void ISR_A() {
  boolean stateA = digitalRead(encoder0pinA) == HIGH;
  boolean stateB = digitalRead(encoder0pinB) == HIGH;
  if ((!stateA && stateB) || (stateA && !stateB)) counter++;
  else counter--;
}

// Interrupt Service Routine for encoder B (pin 3)
void ISR_B() {
  boolean stateA = digitalRead(encoder0pinA) == HIGH;
  boolean stateB = digitalRead(encoder0pinB) == HIGH;
  if ((!stateA && !stateB) || (stateA && stateB)) counter++;
  else counter--;
}
