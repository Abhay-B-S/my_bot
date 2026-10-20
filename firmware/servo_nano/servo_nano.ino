// servo_nano: moves up to 3 hobby servos (SG90 / 9G) from text commands over USB serial.
// Board: Arduino Nano. Serial: 57600 baud, end every command with Enter (CR or LF).
//
// Commands:
//   s <n> <angle>   move servo n (0, 1 or 2) to angle 5..175, slowly      -> "OK <angle>"
//   p <a0> <a1>     move servo 0 AND servo 1 TOGETHER (both energised at once: needs a strong supply!) -> "OK <a0> <a1>"
//   v <ms>          set the speed: milliseconds per degree (3..60)         -> "OK <ms>"
//   g <n>           report the last angle sent to servo n (-1 = never)     -> "<angle>"
//   d <n>           stop sending pulses to servo n (it goes limp)          -> "OK"
//   ?               "servo_nano ready"
//
// Wiring (servo n -> Nano pin): servo 0 = D9, servo 1 = D10, servo 2 = D11.
//
// GENTLE START (v2): a hobby servo cannot tell us where it is. The last angle of each servo is therefore
// remembered in the Nano's EEPROM (written once when a move ends). On the first command after a restart the
// servo's pulses start AT THAT REMEMBERED ANGLE, so a servo nobody has turned by hand does not jump (a jump is
// a big current surge). Only then does it move gently, one degree at a time. First use: 90 degrees is assumed.
// A servo is only "attached" (given pulses) after its first command, so servos stay quiet when the Nano starts
// or when the Pi opens the serial port (which restarts the Nano). Servos always move one at a time.
#include <Servo.h>
#include <EEPROM.h>

const int NUM_SERVOS = 3;
const int SERVO_PINS[NUM_SERVOS] = {9, 10, 11};
const int MIN_ANGLE = 5;            // stay a little away from the mechanical ends (they make servos buzz)
const int MAX_ANGLE = 175;
const int DEFAULT_START = 90;       // assumed position when the EEPROM holds nothing yet
const int SETTLE_MS = 400;          // pause after the pulses start, before moving

int stepDelayMs = 15;               // milliseconds per degree (a full 60-degree swing takes about 0.9 s)

Servo servos[NUM_SERVOS];
int angle[NUM_SERVOS] = {-1, -1, -1};   // -1 = not attached yet

char line[32];
byte lineLen = 0;

int rememberedAngle(int n) {
  int a = EEPROM.read(n);
  return (a >= MIN_ANGLE && a <= MAX_ANGLE) ? a : DEFAULT_START;   // blank EEPROM reads 255
}

void ensureAttached(int n) {
  if (angle[n] < 0) {                   // first command since the restart
    angle[n] = rememberedAngle(n);
    servos[n].write(angle[n]);          // set the pulse BEFORE attaching, so the servo is told "stay here"
    servos[n].attach(SERVO_PINS[n]);
    delay(SETTLE_MS);
  }
}

void moveBoth(int target0, int target1) {   // servo 0 and servo 1 step together, one degree at a time
  target0 = constrain(target0, MIN_ANGLE, MAX_ANGLE);
  target1 = constrain(target1, MIN_ANGLE, MAX_ANGLE);
  ensureAttached(0);
  ensureAttached(1);
  while (angle[0] != target0 || angle[1] != target1) {
    if (angle[0] != target0) {
      angle[0] += (target0 > angle[0]) ? 1 : -1;
      servos[0].write(angle[0]);
    }
    if (angle[1] != target1) {
      angle[1] += (target1 > angle[1]) ? 1 : -1;
      servos[1].write(angle[1]);
    }
    delay(stepDelayMs);
  }
  EEPROM.update(0, angle[0]);
  EEPROM.update(1, angle[1]);
}

void moveServo(int n, int target) {
  target = constrain(target, MIN_ANGLE, MAX_ANGLE);
  ensureAttached(n);
  int step = (target > angle[n]) ? 1 : -1;
  while (angle[n] != target) {          // then move gently, one degree at a time
    angle[n] += step;
    servos[n].write(angle[n]);
    delay(stepDelayMs);
  }
  EEPROM.update(n, angle[n]);           // remember where it ended (writes only when the value changed)
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
  if (cmd == 'p' && got == 3) {         // here n and a are the two TARGET ANGLES
    moveBoth(n, a);
    Serial.print("OK ");
    Serial.print(angle[0]);
    Serial.print(" ");
    Serial.println(angle[1]);
    return;
  }
  if (cmd == 'v' && got >= 2) {
    stepDelayMs = constrain(n, 3, 60);
    Serial.print("OK ");
    Serial.println(stepDelayMs);
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
    Serial.println("ERR use: s <n> <angle>, p <a0> <a1>, v <ms>, g <n>, d <n>, ?");
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
