#include <Arduino.h>
#include <map>
#include <Preferences.h>
#include <vector>
#include "C:\Users\user\Documents\PlatformIO\Projects\robotic arm v1\lib\SCServo\SCServo.h"
#include <cmath>
#include "LOGGER.h"

#define SERVO_MOVEMENT_THRESHOLD 50
#define SAFE_SERVO_MODE true

constexpr int numOfServos = 3;
constexpr float L1 = 13.85;
constexpr float L2 = 9.9;
constexpr float Hb = 11.1f;
constexpr float portataMax =  L1+L2;
constexpr float portataMin = L1-L2;
constexpr float RAD_TO_STEPS = 4095.0f / (2.0f * M_PI);

void initPeripherals();
void initServos();
void getServoID(int& servoVar, const char* servoName);
std::vector<int> getServoList();
void configServo(const int& id, int& max, int& min, const char* servoName);
void logServo(const int& id, const char* servoName);
bool moveServo(const int& id, const int& max, const int& min, const int& speed, const int& acc, int pos, const bool& step);
void calcIK(int& posBase, int& posShoulder, int& posElbow, float x, float y, float z);

SMS_STS st;
Preferences pref;

int servoBaseID;
int servoShoulderID;
int servoElbowID;

int servoBaseMin;
int servoBaseMax;
int servoShoulderMin;
int servoShoulderMax;
int servoElbowMin;
int servoElbowMax;

void setup() {
    initPeripherals();
    initServos();
}

void loop() {
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if(line.length() == 0) return;

        float x, y, z;
        int parsed = sscanf(line.c_str(), "%f %f %f", &x, &y, &z);

        if (parsed == 3) {
            int posBase = 0, posShoulder = 0, posElbow = 0;

            calcIK(posBase, posShoulder, posElbow, x, y, z);

            LOGF("X=%.2f Y=%.2f Z=%.2f\n", x, y, z);
            LOGF("Base=%d Shoulder=%d Elbow=%d\n", posBase, posShoulder, posElbow);

            moveServo(servoBaseID, servoBaseMax, servoBaseMin, 1200, 50, posBase, true);
            moveServo(servoShoulderID, servoShoulderMax, servoShoulderMin, 1200, 50, posShoulder, true);
            moveServo(servoElbowID, servoElbowMax, servoElbowMin, 1200, 50, posElbow, true);
        } else {
            LOG("Errore, scrivi: x y z");
        }
    }
}

void initPeripherals() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, 18, 19);
    st.pSerial = &Serial1;
    pref.begin("servo", false);
    LOG("Peripherals inizializzate");
    delay(2000);
}

void initServos() {
    servoBaseID = pref.getInt("baseID", -1);
    servoShoulderID = pref.getInt("shoulderID", -1);
    servoElbowID = pref.getInt("elbowID", -1);

    if (servoBaseID == -1) {
        getServoID(servoBaseID, "baseID"); pref.putInt("baseID", servoBaseID);
    }
    if (servoShoulderID == -1) {
        getServoID(servoShoulderID, "shoulderID"); pref.putInt("shoulderID", servoShoulderID);
    }
    if (servoElbowID == -1) {
        getServoID(servoElbowID, "elbowID"); pref.putInt("elbowID", servoElbowID);
    }

    st.unLockEprom(servoBaseID);
    servoBaseMin = st.readWord(servoBaseID, 0x9);
    servoBaseMax = st.readWord(servoBaseID, 0xB);
    st.LockEprom(servoBaseID);

    st.unLockEprom(servoShoulderID);
    servoShoulderMin = st.readWord(servoShoulderID, 0x9);
    servoShoulderMax = st.readWord(servoShoulderID, 0xB);
    st.LockEprom(servoShoulderID);

    st.unLockEprom(servoElbowID);
    servoElbowMin = st.readWord(servoElbowID, 0x9);
    servoElbowMax = st.readWord(servoElbowID, 0xB);
    st.LockEprom(servoElbowID);

    if (servoBaseMax == 4095 || servoBaseMin == 0) {
        configServo(servoBaseID, servoBaseMax, servoBaseMin, "Base");
    }

    if (servoShoulderMax == 4095 || servoShoulderMin == 0) {
        configServo(servoShoulderID, servoShoulderMax, servoShoulderMin, "Shoulder");
    }

    if (servoElbowMax == 4095 || servoElbowMin == 0) {
        configServo(servoElbowID, servoElbowMax, servoElbowMin, "Elbow");
    }

    st.EnableTorque(servoBaseID, true);
    st.EnableTorque(servoShoulderID, true);
    st.EnableTorque(servoElbowID, true);
    Serial.println("Servo inizializzati");
}

void getServoID(int& servoVar, const char* servoName) {
    std::vector<int> servos = getServoList();
    std::map<int,int> initialPositions;

    for (int id : servos) {
        initialPositions[id] = st.ReadPos(id); st.EnableTorque(id, false);
    }

    Serial.printf("Muovi il servo: %s\n", servoName);

    while (servoVar == -1) {
        for (int id : servos) {

            if (abs(st.ReadPos(id) - initialPositions[id]) > SERVO_MOVEMENT_THRESHOLD) {
                servoVar = id;
                Serial.printf("Servo %s trovato con ID: %d\n", servoName, servoVar);
                break;
            }

        }
    }
    delay(3000);
}

void configServo(const int& id, int& max, int& min, const char* servoName) {
    Serial.println("Digita [q] per impostare il MIDDLE POINT");
    st.EnableTorque(id, false);

    while (!Serial.readString().equals("q")) {
        st.CalibrationOfs(id);
    }

    Serial.printf("Muovi il servo: %s ai limiti\n", servoName);

    max = st.ReadPos(id);
    min = max;

    while (!Serial.readString().equals("q")) {
        int pos = st.ReadPos(id);
        if (pos > max) max=pos;
        if (pos < min) min=pos;
        Serial.printf("Max: %d Min: %d\n", max, min);
    }

    st.unLockEprom(id);
    st.writeWord(id, 0x9, min);
    st.writeWord(id, 0xB, max);
    st.LockEprom(id);

}

bool moveServo(const int& id, const int& max, const int& min, const int& speed, const int& acc, int pos, const bool& step) {
    pos = step ? pos : pos*4095/360;

    if (pos>max) pos=max;
    if (pos<min) pos=min;

    st.WritePosEx(id, pos, speed, acc);

    return (pos==max || pos==min);
}

std::vector<int> getServoList() {
    static std::vector<int> servos;

    if (servos.empty()) {

        for (int i = 0; i <= 254; i++) {
            int p = st.Ping(i);
            if (p != -1) { servos.push_back(p); }
            if (servos.size() >= numOfServos) break;
            delay(10);
        }

    }

    return servos;
}

void calcIK(int& posBase, int& posShoulder, int& posElbow, float x, float y, float z) {
    float zRel = z - Hb;

    float offsetR = 3;

    float R = sqrt(x*x + y*y) - offsetR;
    float D = sqrt(R*R + zRel*zRel);

    constexpr float MAX_ELBOW_PHYSICAL_DEG = 130.0f;
    float maxElbowRad = MAX_ELBOW_PHYSICAL_DEG * M_PI / 180.0f;

    float Dmin_physical = sqrtf(L1 * L1 + L2 * L2 + 2.0f * L1 * L2 * cosf(maxElbowRad));

    float Dmax = L1 + L2;
    float Dmin = Dmin_physical;

    if (D > Dmax) {
        float scale = Dmax / D;
        R *= scale;
        zRel *= scale;
        D = Dmax;
    } else if (D < Dmin) {
        if (D > 1e-6f) {
            float scale = Dmin / D;
            R *= scale;
            zRel *= scale;
        } else {
            R = Dmin;
            zRel = 0.0f;
        }
        D = Dmin;
    }

    float theta1 = atan2f(y, x);

    float cosTheta3 = (D * D - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cosTheta3 = fminf(1.0f, fmaxf(-1.0f, cosTheta3));

    float theta3 = acosf(cosTheta3);

    float alpha = atan2f(zRel, R);
    float beta = atan2f(L2 * sinf(theta3), L1 + L2 * cosf(theta3));

    float theta2 = alpha + beta;

    posBase = 2048 + (int)lroundf(theta1 * RAD_TO_STEPS);
    posShoulder = 2048 + (int)lroundf((theta2 - M_PI_2) * RAD_TO_STEPS);
    posElbow = 2048 - (int)lroundf(theta3 * RAD_TO_STEPS);
}