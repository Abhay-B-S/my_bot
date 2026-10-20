// servo_nano: moves up to 3 hobby servos (SG90 / 9G) from text commands over USB serial.
// Board: Arduino Nano. Serial: 57600 baud, end every command with Enter (CR or LF).
//
// Commands:
//   s <n> <angle>   move servo n (0, 1 or 2) to angle 0..180, slowly   -> "OK <angle>"
//   g <n>           report the last angle sent to servo n (-1 = never)  -> "<angle>"
//   d <n>           stop sending pulses to servo n (it goes limp)       -> "OK"
//   ?               "servo_nano ready"
//
// Wiring (servo n -> Nano pin): servo 0 = D9, servo 1 = D10, servo 2 = D11.
// A servo is only "attached" (given pulses) after its first command, so an unused or not-yet-commanded
// servo stays quiet when the Nano starts or when the Pi opens the serial port (which resets the Nano).
#include <Servo.h>

const int NUM_SERVOS = 3;
const int SERVO_PINS[NUM_SERVOS] = {9, 10, 11};
const int MIN_ANGLE = 5;            // stay a little away from the mechanical ends (they make servos buzz)
const int MAX_ANGLE = 175;
const int STEP_DELAY_MS = 12;       // 1 degree every 12 ms: a full sweep takes about 2 s (gentle on the supply)

Servo servos[NUM_SERVOS];
int angle[NUM_SERVOS] = {-1, -1, -1};   // -1 = not attached yet

char line[32];
byte lineLen = 0;

void moveServo(int n, int target) {
  target = constrain(target, MIN_ANGLE, MAX_ANGLE);
  if (angle[n] < 0) {                   // first command: attach and go straight there
    servos[n].attach(SERVO_PINS[n]);
    servos[n].write(target);
    angle[n] = target;
    return;
  }
  int step = (target > angle[n]) ? 1 : -1;
  while (angle[n] != target) {          // then move gently, one degree at a time
    angle[n] += step;
    servos[n].write(angle[n]);
    delay(STEP_DELAY_MS);
  }
}

void handleLine(char *text) {
  char cmd = 0;
  int n = -1, a = 0;
  int got = sscanf(text, " %c %d %d", &cmd, &n, &a);
  if (got < 1) {
    return;                             // empty line
  }
  if (cmd == '?') {
    Serial.println("servo_nano ready");
    return;
  }
  if (got < 2 || n < 0 || n >= NUM_SERVOS) {
    Serial.println("ERR servo number must be 0, 1 or 2");
    return;
  }
  if (cmd == 's' && got == 3) {
    moveServo(n, a);
    Serial.print("OK ");
    Serial.println(angle[n]);
  } else if (cmd == 'g') {
    Serial.println(angle[n]);
  } else if (cmd == 'd') {
    servos[n].detach();
    angle[n] = -1;
    Serial.println("OK");
  } else {
    Serial.println("ERR use: s <n> <angle>, g <n>, d <n>, ?");
  }
}

void setup() {
  Serial.begin(57600);
  Serial.println("servo_nano ready");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        line[lineLen] = 0;
        handleLine(line);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}
