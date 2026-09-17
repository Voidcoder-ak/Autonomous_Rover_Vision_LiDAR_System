#include <Arduino.h>

#define ENA 25 // Left motor speed (PWM)
#define IN1 26 // Left motor direction
#define IN2 27
#define ENB 33 // Right motor speed (PWM)
#define IN3 32 // Right motor direction
#define IN4 18

const int MIN_PWM = 10;
const int RAMP_STEP = 5;
const unsigned long COMMAND_TIMEOUT = 200;
unsigned long lastCommandTime = 0;
unsigned long lastRampTime = 0;
const unsigned long RAMP_TIME = 20;
int leftCurrentPWM = 0;
int rightCurrentPWM = 0;
int leftTargetPWM = 0;
int rightTargetPWM = 0;

bool isValidNumber(String s)
{
    if (s.length() == 0)
        return false;

    int startIndex = 0;

    if (s[0] == '-')
    {
        startIndex = 1;
        if (s.length() <= 1)
        {
            return false;
        }
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

    // brake: Complete stopping
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

void setup()
{
    Serial.begin(115200); // Opens a "conversation channel" at 115200 bits/sec
    // Both sides (ESP32 and your laptop) must agree on this speed

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcAttach(ENA, 5000, 8); // pin, frequency (Hz), resolution (bits)
    ledcAttach(ENB, 5000, 8);

    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
}

void updateRamping()
{
    if(millis() - lastRampTime < RAMP_TIME){
        return;
    }
    lastRampTime = millis();
    if (leftCurrentPWM == leftTargetPWM)
    {
        // do Nothing
    }
    else if (leftTargetPWM < leftCurrentPWM)
    {
        leftCurrentPWM -= RAMP_STEP;
        if (leftCurrentPWM < leftTargetPWM)
        {
            leftCurrentPWM = leftTargetPWM;
        }
        setMotor(ENA, IN1, IN2, leftCurrentPWM);
    }
    else
    {
        leftCurrentPWM += RAMP_STEP;
        if (leftCurrentPWM > leftTargetPWM)
        {
            leftCurrentPWM = leftTargetPWM; // snap to exact target
        }
        setMotor(ENA, IN1, IN2, leftCurrentPWM);
    }
    if (rightCurrentPWM == rightTargetPWM)
    {
        // do nothing
    }
    else if (rightTargetPWM < rightCurrentPWM)
    {
        rightCurrentPWM -= RAMP_STEP;
        if (rightCurrentPWM < rightTargetPWM)
        {
            rightCurrentPWM = rightTargetPWM;
        }
        setMotor(ENB, IN3, IN4, rightCurrentPWM);
    }
    else
    {
        rightCurrentPWM += RAMP_STEP;
        if (rightCurrentPWM > rightTargetPWM)
        {
            rightCurrentPWM = rightTargetPWM;
        }
        setMotor(ENB, IN3, IN4, rightCurrentPWM);
    }
}

void loop()
{
    if (millis() - lastCommandTime > COMMAND_TIMEOUT)
    {
        leftTargetPWM = 0;
        rightTargetPWM = 0;
    }
    if (Serial.available())
    {                                               // "Has any data arrived?"
        String line = Serial.readStringUntil('\n'); // Read until newline character
        line.trim();

        int lIndex = line.indexOf("L:");
        int rIndex = line.indexOf(",R:");

        if (lIndex != -1 && rIndex != -1)
        {
            if (!isValidNumber(line.substring(lIndex + 2, rIndex)) || !isValidNumber(line.substring(rIndex + 3)))
            {
                Serial.println("Invalid Value: Provide integer values");
            }
            else
            {
                int leftVal = line.substring(lIndex + 2, rIndex).toInt();
                int rightVal = line.substring(rIndex + 3).toInt();

                leftTargetPWM = leftVal;
                rightTargetPWM = rightVal;

                // setMotor(ENA, IN1, IN2, leftVal);
                // setMotor(ENB, IN3, IN4, rightVal);

                lastCommandTime = millis();

                Serial.print("Left: ");
                Serial.print(leftVal);
                Serial.print(" | Right: ");
                Serial.println(rightVal);
            }
        }
        else
        {
            Serial.println("Invalid Format. Use L:<val>,R:<val>");
        }
    }
    updateRamping();
}