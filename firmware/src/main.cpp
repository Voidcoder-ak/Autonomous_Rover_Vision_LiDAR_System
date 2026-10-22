// Active code: IR line sensor + motor control + serial override
// CHANGED 1-Oct: added the laptop SPEED link for the LiDAR stop test (B1).
//   S:<0..100>  from the laptop (lidar_stop sends S:0 = stop, S:100 = go every 100 ms)
//   - scales baseSpeed by the percentage; PID/P steering stays on the ESP32
//   - S:0 stops IMMEDIATELY (no ramp-down), so stopping distance stays short
//   - FAILSAFE: once the laptop link is active, no S: for 200 ms -> stop
//   - S:<n> with n > 0 arms line following (if calibrated), so the laptop can start it
//   PCT:<0..100>  same speed scaling, typed by hand, NO failsafe (for the B2 sharp-turn test)
//   Search for "CHANGED 1-Oct" to see every change (8 places).
// Fixes: calibration check (no auto-drive when uncalibrated), no divide-by-zero in map(),
//        armed/disarmed state (boots stopped), non-blocking-ish serial, cleaner mode logging.

#include <Arduino.h>
#include <Preferences.h>

// ---------------- Motor pins ----------------
#define ENA 14
#define IN1 27
#define IN2 26
#define ENB 22
#define IN3 25
#define IN4 23

int MIN_PWM = 180; // lowest PWM that actually turns the wheels; tune live with MIN:<value>
const int RAMP_STEP = 20;
const unsigned long RAMP_TIME = 20;
unsigned long lastRampTime = 0;

int leftCurrentPWM = 0;
int rightCurrentPWM = 0;
int leftTargetPWM = 0;
int rightTargetPWM = 0;

// ---------------- Modes ----------------
const unsigned long OVERRIDE_TIMEOUT = 500;
unsigned long lastCommandTime = 0;
bool isOverrideMode = false;
bool isArmed = false; // boots DISARMED: auto line following only after "GO"

// CHANGED 1-Oct (1/8): speed link state
int speedPct = 100;           // 0..100, multiplies baseSpeed
bool speedLinkActive = false; // true once the laptop has sent an S: command
unsigned long lastSpeedCmdTime = 0;
const unsigned long SPEED_FAILSAFE_MS = 200; // laptop silent this long -> stop

// ---------------- Line sensor ----------------
Preferences preferences;
const int NUM_SENSORS = 5;
const int sensorPins[NUM_SENSORS] = {32, 33, 34, 35, 36};

int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
int normalizedValues[NUM_SENSORS];

bool isCalibrating = false;
bool isCalibrated = false;
unsigned long calibStartTime = 0;
const unsigned long CALIBRATION_DURATION = 5000;
const int MIN_CAL_RANGE = 500; // min (max - min) gap per sensor; tune after seeing real values

// ---------------- Tuning ----------------
float Kd = 0.1; // CHANGED 1-Oct (13): D term; tune live with KD:<value>
int prevError = 0;
float Kp = 0.04;     // was 0.04; tune live with KP:<value>
int baseSpeed = 140; // was 180; tune live with SPD:<value>

// ======================================================

bool isValidNumber(String s)
{
    if (s.length() == 0)
        return false;
    int startIndex = 0;
    if (s[0] == '-')
    {
        startIndex = 1;
        if (s.length() <= 1)
            return false;
    }
    for (int i = startIndex; i < s.length(); i++)
    {
        if (!isdigit(s[i]))
            return false;
    }
    return true;
}

void setMotor(int pwmPin, int in1, int in2, int value)
{
    value = constrain(value, -255, 255);

    if (value == 0)
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        ledcWrite(pwmPin, 0);
        return;
    }

    if (value > 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        value = -value;
    }

    int effectivePwm = map(value, 1, 255, MIN_PWM, 255);
    ledcWrite(pwmPin, effectivePwm);
}

void stopTargets()
{
    leftTargetPWM = 0;
    rightTargetPWM = 0;
}

// CHANGED 1-Oct (2/8): stop NOW, skipping the ramp-down.
// Why: ramping 120 -> 0 in steps of 5 every 20 ms takes ~0.5 s, and the
// rover would keep rolling toward the obstacle the whole time.
void stopImmediately()
{
    stopTargets();
    leftCurrentPWM = 0;
    rightCurrentPWM = 0;
    setMotor(ENA, IN1, IN2, 0);
    setMotor(ENB, IN3, IN4, 0);
}

void updateRamping()
{
    if (millis() - lastRampTime < RAMP_TIME)
        return;
    lastRampTime = millis();

    // Left motor
    if (leftTargetPWM < leftCurrentPWM)
    {
        leftCurrentPWM -= RAMP_STEP;
        if (leftCurrentPWM < leftTargetPWM)
            leftCurrentPWM = leftTargetPWM;
    }
    else if (leftTargetPWM > leftCurrentPWM)
    {
        leftCurrentPWM += RAMP_STEP;
        if (leftCurrentPWM > leftTargetPWM)
            leftCurrentPWM = leftTargetPWM;
    }
    setMotor(ENA, IN1, IN2, leftCurrentPWM);

    // Right motor
    if (rightTargetPWM < rightCurrentPWM)
    {
        rightCurrentPWM -= RAMP_STEP;
        if (rightCurrentPWM < rightTargetPWM)
            rightCurrentPWM = rightTargetPWM;
    }
    else if (rightTargetPWM > rightCurrentPWM)
    {
        rightCurrentPWM += RAMP_STEP;
        if (rightCurrentPWM > rightTargetPWM)
            rightCurrentPWM = rightTargetPWM;
    }
    setMotor(ENB, IN3, IN4, rightCurrentPWM);
}

// ---------------- Calibration ----------------

// true only if EVERY sensor saw a big enough difference between line and floor.
// Catches: first boot (min=4095, max=0 -> negative gap) and a sensor that
// never crossed the line (min == max -> would also divide by zero in map()).
bool checkCalibration()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if ((maxValues[i] - minValues[i]) <= MIN_CAL_RANGE)
            return false;
    }
    return true;
}

void printCalibration()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print("S");
        Serial.print(i);
        Serial.print(" min=");
        Serial.print(minValues[i]);
        Serial.print(" max=");
        Serial.print(maxValues[i]);
        Serial.print(" gap=");
        Serial.println(maxValues[i] - minValues[i]);
    }
}

void loadCalibrationParams()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        String minKey = "min" + String(i);
        String maxKey = "max" + String(i);
        minValues[i] = preferences.getInt(minKey.c_str(), 4095);
        maxValues[i] = preferences.getInt(maxKey.c_str(), 0);
    }
    isCalibrated = checkCalibration();
}

void startCalibration()
{
    Serial.println("Calibration started: sweep the rover across the line for 5 s");
    stopTargets(); // never drive on stale targets while calibrating
    isArmed = false;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        minValues[i] = 4095;
        maxValues[i] = 0;
    }
    isCalibrating = true;
    isCalibrated = false;
    calibStartTime = millis();
}

void finishCalibration()
{
    isCalibrating = false;
    printCalibration();

    if (!checkCalibration())
    {
        isCalibrated = false;
        Serial.println("Calibration FAILED: some sensor never saw both line and floor. Not saved. Run CAL again.");
        return;
    }

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        String minKey = "min" + String(i);
        String maxKey = "max" + String(i);
        preferences.putInt(minKey.c_str(), minValues[i]);
        preferences.putInt(maxKey.c_str(), maxValues[i]);
    }
    isCalibrated = true;
    Serial.println("Calibration complete and saved. Send GO to start line following.");
}

void resetCalibration()
{
    preferences.clear();
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        minValues[i] = 4095;
        maxValues[i] = 0;
    }
    isCalibrated = false;
    isArmed = false;
    stopTargets();
    Serial.println("Calibration reset. Run CAL before GO.");
}

// ---------------- Line following ----------------

int calculateLineError()
{
    long weightedSum = 0;
    long sum = 0;
    int weights[NUM_SENSORS] = {-2000, -1000, 0, 1000, 2000};

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        // Assumes the line reads LOW after normalization.
        // If the rover steers away from the line, use: int lineWeight = normalizedValues[i];
        int lineWeight = 1000 - normalizedValues[i];
        if (lineWeight > 200)
        {
            weightedSum += (long)lineWeight * weights[i];
            sum += lineWeight;
        }
    }

    if (sum == 0)
        return 9999; // line lost
    return (int)(weightedSum / sum);
}

void runAutoLineFollower(int error)
{
    if (error == 9999)
    {
        stopTargets();
        return;
    }

    // CHANGED 1-Oct (3/8): speed = baseSpeed x speedPct/100; steering is untouched.
    // The correction is scaled too, so slowing down doesn't make the turns twitchy.
    int base = baseSpeed * speedPct / 100;
    int dErr = error - prevError; // how fast the line is moving under the sensor
    prevError = error;
    int correction = (error * Kp + dErr * Kd) * speedPct / 100; // CHANGED 1-Oct (13): was P only
    leftTargetPWM = constrain(base + correction, -255, 255);
    rightTargetPWM = constrain(base - correction, -255, 255);
}

// ---------------- Serial commands ----------------
// CAL    start calibration (sweep across the line for 5 s)
// RESET  clear saved calibration
// GO     arm auto line following (only if calibrated)
// STOP   disarm and stop motors
// SENS   print normalized sensor values once (for checking line polarity)
// L:<v>,R:<v>  manual override, -255..255, falls back after 500 ms of silence

void printSensors()
{
    Serial.print("Norm: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(normalizedValues[i]);
        Serial.print(i < NUM_SENSORS - 1 ? " " : "");
    }
    Serial.print(" | error=");
    Serial.println(calculateLineError());
}

// Collects characters until Enter (\n or \r). Never blocks, and works with terminals
// that send one character per keypress (like pio device monitor).
String serialBuffer = "";
const unsigned int MAX_LINE_LENGTH = 64;

bool readSerialLine(String &out)
{
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (serialBuffer.length() > 0)
            {
                out = serialBuffer;
                serialBuffer = "";
                return true;
            }
        }
        else
        {
            serialBuffer += c;
            if (serialBuffer.length() > MAX_LINE_LENGTH)
                serialBuffer = ""; // garbage protection
        }
    }
    return false;
}

void handleSerial()
{
    String line;
    if (!readSerialLine(line))
        return;

    line.trim();
    if (line.length() == 0)
        return;

    if (line == "CAL")
    {
        startCalibration();
    }
    else if (line == "RESET")
    {
        resetCalibration();
    }
    else if (line == "GO")
    {
        if (isCalibrated)
        {
            isArmed = true;
            Serial.println("ARMED: auto line following");
        }
        else
        {
            Serial.println("Refused: not calibrated. Run CAL first.");
        }
    }
    else if (line == "STOP")
    {
        isArmed = false;
        isOverrideMode = false;
        speedLinkActive = false; // CHANGED 1-Oct (4/8): STOP also ends the laptop link
        stopImmediately();       // CHANGED 1-Oct (4/8): was stopTargets() (slow ramp-down)
        Serial.println("STOPPED");
    }
    // CHANGED 1-Oct (5/8): S:<0..100> from the laptop (heartbeat every 100 ms)
    else if (line.startsWith("S:"))
    {
        String v = line.substring(2);
        if (!isValidNumber(v) || v.toInt() < 0 || v.toInt() > 100)
            return; // garbage never becomes a speed (and no reply: the laptop only writes)
        speedPct = v.toInt();
        lastSpeedCmdTime = millis();
        if (!speedLinkActive)
        {
            speedLinkActive = true;
            Serial.println("Laptop link: ACTIVE (failsafe on)");
        }
        if (speedPct == 0)
        {
            stopImmediately();
        }
        else if (!isArmed && isCalibrated && !isCalibrating)
        {
            isArmed = true; // the laptop program starting = "GO"
            Serial.println("ARMED by laptop");
        }
    }
    // CHANGED 1-Oct (6/8): PCT:<0..100> typed by hand, no failsafe (B2 sharp-turn test)
    else if (line.startsWith("PCT:"))
    {
        String v = line.substring(4);
        if (isValidNumber(v))
            speedPct = constrain(v.toInt(), 0, 100);
        Serial.print("speedPct = ");
        Serial.println(speedPct);
    }
    else if (line == "SENS")
    {
        if (isCalibrated)
            printSensors();
        else
            Serial.println("Not calibrated");
    }
    // Live tuning, no reflash needed: KP:0.06  SPD:120  TUNE (print current)
    else if (line.startsWith("KP:"))
    {
        float v = line.substring(3).toFloat();
        if (v > 0 && v < 1.0)
            Kp = v;
        Serial.print("Kp = ");
        Serial.println(Kp, 4);
    }
    else if (line.startsWith("SPD:"))
    {
        String s = line.substring(4);
        if (isValidNumber(s))
            baseSpeed = constrain(s.toInt(), 0, 255);
        Serial.print("baseSpeed = ");
        Serial.println(baseSpeed);
    }
    else if (line.startsWith("MIN:"))
    {
        String s = line.substring(4);
        if (isValidNumber(s))
            MIN_PWM = constrain(s.toInt(), 0, 250);
        Serial.print("MIN_PWM = ");
        Serial.println(MIN_PWM);
    }
    else if (line == "TUNE")
    {
        Serial.print("Kp = ");
        Serial.print(Kp, 4);
        Serial.print(" | baseSpeed = ");
        Serial.print(baseSpeed);
        Serial.print(" | MIN_PWM = ");
        Serial.print(MIN_PWM);
        Serial.print(" | speedPct = "); // CHANGED 1-Oct (7/8): also show speed %
        Serial.println(speedPct);
    }
    else
    {
        int lIndex = line.indexOf("L:");
        int rIndex = line.indexOf(",R:");

        if (lIndex == -1 || rIndex == -1)
        {
            Serial.println("Unknown command. Use CAL, RESET, GO, STOP, SENS or L:<val>,R:<val>");
            return;
        }

        String lStr = line.substring(lIndex + 2, rIndex);
        String rStr = line.substring(rIndex + 3);

        if (!isValidNumber(lStr) || !isValidNumber(rStr))
        {
            Serial.println("Invalid value: provide integers");
            return;
        }

        if (isCalibrating)
        {
            Serial.println("Ignored: calibrating");
            return;
        }

        leftTargetPWM = lStr.toInt();
        rightTargetPWM = rStr.toInt();
        lastCommandTime = millis();

        if (!isOverrideMode)
        {
            isOverrideMode = true;
            Serial.println("Mode: OVERRIDE"); // printed only on mode change
        }
    }
}

// ======================================================

void setup()
{
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcAttach(ENA, 1000, 8);
    ledcAttach(ENB, 1000, 8);
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);

    for (int i = 0; i < NUM_SENSORS; i++)
        pinMode(sensorPins[i], INPUT);

    preferences.begin("line_sensors", false); // opened once, kept open
    loadCalibrationParams();

    Serial.println(isCalibrated ? "System ready (calibrated). Send GO to start."
                                : "System ready (NOT calibrated). Send CAL first.");
}

void loop()
{
    handleSerial();

    // CHANGED 1-Oct (8/8): FAILSAFE. Once the laptop link is active, silence = danger.
    // lidar_stop sends every 100 ms, so 200 ms means at least one message was missed.
    if (speedLinkActive && (millis() - lastSpeedCmdTime > SPEED_FAILSAFE_MS))
    {
        if (speedPct != 0)
            Serial.println("FAILSAFE: laptop silent > 200 ms, stopping");
        speedPct = 0;
        stopImmediately();
    }

    // Override expired -> leave override mode
    if (isOverrideMode && (millis() - lastCommandTime > OVERRIDE_TIMEOUT))
    {
        isOverrideMode = false;
        stopTargets();
        Serial.println(isArmed ? "Mode: AUTO" : "Mode: IDLE");
    }

    if (isCalibrating)
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            int rawVal = analogRead(sensorPins[i]);
            if (rawVal < minValues[i])
                minValues[i] = rawVal;
            if (rawVal > maxValues[i])
                maxValues[i] = rawVal;
        }
        if (millis() - calibStartTime > CALIBRATION_DURATION)
            finishCalibration();
    }
    else if (isCalibrated)
    {
        // Only normalize with valid min/max -> map() can never divide by zero here
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            int rawVal = constrain(analogRead(sensorPins[i]), minValues[i], maxValues[i]);
            normalizedValues[i] = map(rawVal, minValues[i], maxValues[i], 0, 1000);
        }

        if (isArmed && !isOverrideMode && speedPct > 0) // CHANGED 1-Oct (3/8): speed 0 = no steering either
            runAutoLineFollower(calculateLineError());
        else if (isArmed && !isOverrideMode)
            stopTargets();
    }

    // Not armed and no manual command -> motors stay stopped
    if (!isArmed && !isOverrideMode)
        stopTargets();

    updateRamping();
    delay(5);
}