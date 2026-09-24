#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ============================================================
// AGV STM32 PRODUCTION FIRMWARE
// Nucleo-F411RE + 2 BTS7960 + 2 encoder
//
// Lenh moi tu Jetson:
//   <steer> <speed>\n
//   steer: -20..20, am=re trai, duong=re phai
//   speed: 0..20, 0=dung, duong=di toi
// Moi lenh khac deu STOP ngay. Firmware production khong cho di lui.
//
// IMU BNO055 (I2C, SDA=PB9/D14, SCL=PB8/D15, addr 0x28): chi de BAO
// CAO du lieu roll/pitch/gia toc tuyen tinh qua telemetry - KHONG anh
// huong logic dieu khien dong co/watchdog. Neu khong tim thay IMU luc
// khoi dong, firmware van chay binh thuong (chi bao IMU,NA,NA,NA).
// CHUA TUNG TEST TREN PHAN CUNG THAT - can nap lai va kiem tra ky
// (dac biet watchdog 500ms van phai hoat dong dung) truoc khi tin
// dung cho van hanh that.
// ============================================================

Adafruit_BNO055 bno(55, 0x28, &Wire);
bool imuReady = false;

// ================= MOTOR PINS =================

#define LEFT_RPWM  D9
#define LEFT_LPWM  D3

#define RIGHT_RPWM D10
#define RIGHT_LPWM D11

// ================= ENCODER PINS =================

// Cau hinh da test thanh cong.
#define LEFT_ENCODER_A  D4
#define LEFT_ENCODER_B  D2

#define RIGHT_ENCODER_A D5
#define RIGHT_ENCODER_B D6

// ================= CONTROL CONSTANTS =================

// PWM feed-forward da test tren mat dat.
const int BASE_PWM = 90;
const int PWM_MIN  = 70;
// Gioi han tam thoi cho lan chay an toan dau tien.
// 2026-09-10: tung la 90, trong khi analogWrite chay thang 0..255 - tuc xe
// moi dung 35% dai PWM. Do tren duong: khi be lai het co, banh NGOAI bi ghim
// o PWM 88/90 (het duong day) ma chenh lech toc do hai banh chi 5.7 ticks
// tren ~29, cho ban kinh quay ~16.7 vet banh. Bo PI khong con cho de dua
// banh ngoai chay nhanh hon nua.
//
// BASE_PWM giu nguyen 90, nen feedforward luc di thang KHONG doi (o 33
// ticks van ra ~86) - viec nang tran nay chi cho bo PI them cho de day banh
// ngoai TRONG LUC CUA, khong lam xe chay nhanh hon tren duong thang.
const int PWM_MAX  = 140;

// Tai speed=90, dat muc tieu 40 xung / 100 ms.
// 2026-09-10: tung la 90, khong khop voi SPEED_COMMAND_MAX=25 ngay ben duoi.
// Dau vao toi da la 25 nhung thang quy doi lay moc 90, nen lenh cho banh luon
// nam trong 0..11 cua thang 70. Do tren gia: lenh 9 ticks thi banh quay 35-39
// ticks - ca hai banh vuot xa muc tieu nen bo PI chay kich tran tich phan va
// chenh lech lai gan nhu khong thanh hien thuc: be trai het co (target 3/15,
// ti le 1:5) chi cho ra 33/42 ticks, tuc 1:1.27.
//
// Do la ly do xe troi phai ma be lai het co van khong keo lai duoc: tren duong
// da giu steer=-20 suot 8.4 giay ma do lech con xau di 113 px.
//
// Dat = SPEED_COMMAND_MAX de dai vao 0..25 trai deu len 0..40 ticks (moc
// tham chieu), cho muc tieu nam trong tam voi cua banh va bo PI lam viec
// trong vung tuyen tinh.
const int SPEED_REFERENCE = 25;
const int TARGET_TICKS_AT_REFERENCE = 40;

// 2026-09-04: nang tu 20 -> 25 (khop voi SPEED_MAX moi trong real_car_socket.py).
// 20 ban dau chi de an toan cho lan test dong co that dau tien - da qua
// nhieu buoc kiem chung khac (watchdog, LiDAR, PWM clamp fix) nen nang len.
const int SPEED_COMMAND_MAX = 25;
const int STEER_COMMAND_MAX = 20;

// Do manh cua viec re.
// 70 nghia la o steer=20, do lech toc do hai banh bang 70% toc do co ban.
const int STEER_GAIN_PERCENT = 70;

// Neu xe re nguoc voi lenh, chi doi 1 thanh -1.
const int STEER_SIGN = 1;

// Bu lech co khi cua xe, tinh bang TICK moi banh (khong phai don vi steer).
// Duong = banh trai nhanh hon = keo xe sang PHAI.
// -1 => trai cham 1 tick, phai nhanh 1 tick => chenh -2 tick, keo sang TRAI,
// bang dung STEER_TRIM=-2 ben Python truoc day (do: steer=-2 -> turn=-1 ->
// chenh -2). Python phai dat STEER_TRIM=0 khi dung ban firmware nay, neu
// khong luong bu se bi tinh hai lan.
const int STEER_TRIM_TICKS = -1;

// Bao ve muc tieu qua cao khi vua chay nhanh vua re.
const int MAX_WHEEL_TARGET_TICKS = 70;

const unsigned long CONTROL_INTERVAL_MS = 100;

// 2026-09-12: 200 -> 100 ms. Bang voi CONTROL_INTERVAL_MS, co chu y.
//
// Phep do DO TRE lenh->chuyen dong bi chan boi nhip nay, khong boi cam
// bien. cmd_steer chi ton tai trong log o moi dong telemetry, nen o 200 ms
// thoi diem lenh doi bi ghi tre toi 200 ms va do tre doc duoc thap hon
// thuc te ~100 ms - tren mot dai luong 400-600 ms. Doi con IMU nhanh hon
// (MTi-630R o 95 Hz) KHONG sua duoc dieu do: nut co that nam o phia LENH.
// Ha xuong 100 ms lam luong tu con +-50 ms, va giu lenh voi chuyen dong
// tren CUNG mot dong ho - dieu ma hai thiet bi rieng khong bao gio co.
//
// KHONG ha duoi 100 ms ma khong sua CONTROL_INTERVAL_MS truoc:
// leftDelta/rightDelta chi duoc tinh trong block CONTROL_INTERVAL_MS, nen
// in nhanh hon se IN TRUNG delta cu. Du lieu trung trong nhu du lieu that
// va moi toc do tinh tu no deu sai.
//
// Ngan sach UART: dong ENC hien 106 byte; o 115200 8N1 (1152 byte/s) thi
// 10 Hz dung 9.2% duong truyen. 20 Hz se la 18.4% - con cho - nhung bi
// chan boi ly do delta o tren, khong phai boi baud.
const unsigned long PRINT_INTERVAL_MS   = 100;
const unsigned long TIMEOUT_MS          = 500;

const float KP = 0.45f;
const float KI = 0.015f;
const float INTEGRAL_LIMIT = 600.0f;

// Hieu chinh lech co khi banh trai/phai. Lich su:
// - 2026-09-04: do qua telemetry luc STEER=-20 (khi dang re), TARGET yeu
//   cau phai nhanh gap ~4.5 lan trai nhung DELTA thuc te gan bang nhau ->
//   dat trim=1.30. SO LIEU NAY BI NHIEU boi hieu ung re (steer=-20), khong
//   phai baseline chay thang thuan tuy.
// - 2026-09-08: do lai bang wheel_calibration.py voi STEER=0 (chay thang
//   thuan, TARGET trai=phai) o 4 muc SPEED (10/15/20/25) - phat hien
//   trim=1.30 da BU QUA TAY: banh phai chay nhanh hon trai ~26% rat nhat
//   quan o moi toc do (ty le do duoc: 1.24-1.27), du TARGET bang nhau.
//   Tinh lai: trim_dung = 1.30 / 1.26 ~= 1.03.
// Nhan them vao rightPWM sau khi PI tinh xong. 1.0 = khong bu. Tang dan
// +-0.05 neu van con lech, dung so lieu DELTA thuc te (khong phai cam
// quan) de kiem tra - uu tien do o STEER=0 (baseline) truoc khi danh gia
// luc re (STEER != 0), 2 phep do khac nhau, dung lan.
const float RIGHT_WHEEL_PWM_TRIM = 1.03f;

// Chieu motor thuc te da test:
// -1 la xe di toi, +1 la xe di lui.
const int FORWARD_MOTOR_DIRECTION  = -1;

// ================= STATE =================

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

// 2026-09-12: so thu tu telemetry, bat dau tu 0 moi lan MCU khoi dong.
// Mot truong duy nhat tra loi duoc hai cau hoi vua khong tra loi duoc:
//   - MAT BAN TIN: so thu tu co lo hong
//   - MCU DA RESET: so thu tu nhay ve gan 0
// Truoc day phia phan tich phai suy "da reset" tu viec bo dem encoder
// GIAM - suy luan sai, vi bo dem co dau (++/-- trong ISR) nen quay banh
// nguoc chieu lam no giam la binh thuong. Chinh bai quay tay de do
// xung/vong cung lam no giam.
// unsigned long 32-bit o 10 Hz thi tran sau ~13.6 nam.
unsigned long telemetrySeq = 0;

long previousLeftCount  = 0;
long previousRightCount = 0;

long leftDelta  = 0;
long rightDelta = 0;

float leftIntegral  = 0.0f;
float rightIntegral = 0.0f;

int leftPWM  = 0;
int rightPWM = 0;

int leftDirection  = 0;
int rightDirection = 0;

int leftTargetTicks  = 0;
int rightTargetTicks = 0;

int requestedSteer = 0;
int requestedSpeed = 0;

// S=stop, V=steer/speed.
char currentMode = 'S';

unsigned long lastCommandTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastPrintTime   = 0;

char serialBuffer[40];
size_t serialIndex = 0;
bool serialDiscarding = false;

// ================= ENCODER ISR =================

void leftEncoderISR()
{
    // Cau hinh dau da test: khi xe di toi, encoder tang duong.
    if (digitalRead(LEFT_ENCODER_B) == HIGH) {
        leftEncoderCount++;
    } else {
        leftEncoderCount--;
    }
}

void rightEncoderISR()
{
    // Cau hinh dau da test: khi xe di toi, encoder tang duong.
    if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
        rightEncoderCount++;
    } else {
        rightEncoderCount--;
    }
}

// ================= LOW-LEVEL MOTOR =================

void setLeftMotor(int value)
{
    value = constrain(value, -255, 255);

    if (value > 0) {
        analogWrite(LEFT_RPWM, value);
        analogWrite(LEFT_LPWM, 0);
    } else if (value < 0) {
        analogWrite(LEFT_RPWM, 0);
        analogWrite(LEFT_LPWM, -value);
    } else {
        analogWrite(LEFT_RPWM, 0);
        analogWrite(LEFT_LPWM, 0);
    }
}

void setRightMotor(int value)
{
    value = constrain(value, -255, 255);

    if (value > 0) {
        analogWrite(RIGHT_RPWM, value);
        analogWrite(RIGHT_LPWM, 0);
    } else if (value < 0) {
        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, -value);
    } else {
        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, 0);
    }
}

void applyMotorOutputs()
{
    setLeftMotor(leftDirection * leftPWM);
    setRightMotor(rightDirection * rightPWM);
}

// ================= PI HELPERS =================

int feedForwardPWM(int targetTicks)
{
    if (targetTicks <= 0) {
        return 0;
    }

    // 2026-09-10: cong thuc cu la
    //     PWM_MIN + (BASE_PWM - PWM_MIN) * t/TREF
    // tuc no BAT DAU TU PWM_MIN=70 ngay ca khi target ~ 0. Do la bu ma sat
    // tinh de banh KHOI DONG duoc, nhung khi can LAM CHAM mot banh dang bi
    // than xe keo di thi no phan tac dung.
    //
    // Do tren duong, be lai het co, speed 12:
    //     TARGET  6/32     PWM 53/81     DELTA 31/37
    // Banh NGOAI dat muc tieu tot (lenh 32, chay 33-37). Banh TRONG duoc
    // lenh 6 ticks nhung quay 24-31 ticks - khong he cham lai. Vi:
    //     feedForwardPWM(6) = 73        <- cong thuc cu
    //     KP*error          = -8.1
    //     KI*integral(tran) = -9.0
    //     -> PWM = 55.9 (do duoc 53)
    // Bo PI chi tru duoc 17 nac tren tong 73, ma can tru toi 60. Nen banh
    // trong bi DRIVE o ~53 PWM trong khi le ra phai duoc tha tu do.
    //
    // Cho duong dac tuyen di qua goc toa do: t=40 van ra 90 nhu cu (diem
    // tham chieu khong doi), nhung t=6 ra 13 thay vi 73. Bu ma sat tinh de
    // lai cho khau PI: neu banh khong quay thi error duong lon, PI se tu
    // day PWM len qua nguong ma sat trong 1-2 giay.
    float pwm =
        BASE_PWM *
        ((float)targetTicks / (float)TARGET_TICKS_AT_REFERENCE);

    return constrain((int)(pwm + 0.5f), 0, PWM_MAX);
}

int updateOneWheelPI(
    int targetTicks,
    long measuredTicks,
    float &integral
)
{
    if (targetTicks <= 0) {
        integral = 0.0f;
        return 0;
    }

    float error = (float)targetTicks - (float)measuredTicks;
    integral += error;
    integral = constrain(
        integral,
        -INTEGRAL_LIMIT,
        INTEGRAL_LIMIT
    );

    float output =
        (float)feedForwardPWM(targetTicks) +
        KP * error +
        KI * integral;

    return constrain((int)(output + 0.5f), 0, PWM_MAX);
}

void resetController()
{
    noInterrupts();
    previousLeftCount  = leftEncoderCount;
    previousRightCount = rightEncoderCount;
    interrupts();

    leftDelta  = 0;
    rightDelta = 0;

    leftIntegral  = 0.0f;
    rightIntegral = 0.0f;

    leftPWM  = feedForwardPWM(leftTargetTicks);
    rightPWM = feedForwardPWM(rightTargetTicks);

    lastControlTime = millis();
}

void stopMotors()
{
    leftDirection  = 0;
    rightDirection = 0;

    leftTargetTicks  = 0;
    rightTargetTicks = 0;

    leftPWM  = 0;
    rightPWM = 0;

    leftDelta  = 0;
    rightDelta = 0;

    leftIntegral  = 0.0f;
    rightIntegral = 0.0f;

    requestedSteer = 0;
    requestedSpeed = 0;
    currentMode = 'S';

    setLeftMotor(0);
    setRightMotor(0);
}

// ================= CONTINUOUS COMMAND =================

int speedToBaseTarget(int speedMagnitude)
{
    long target =
        (long)speedMagnitude *
        TARGET_TICKS_AT_REFERENCE /
        SPEED_REFERENCE;

    return constrain(
        (int)target,
        0,
        MAX_WHEEL_TARGET_TICKS
    );
}

void startVelocityCommand(int steer, int speed)
{
    steer = constrain(
        steer,
        -STEER_COMMAND_MAX,
        STEER_COMMAND_MAX
    );

    speed = constrain(
        speed,
        0,
        SPEED_COMMAND_MAX
    );

    lastCommandTime = millis();

    if (speed == 0) {
        stopMotors();
        return;
    }

    if (
        currentMode == 'V' &&
        steer == requestedSteer &&
        speed == requestedSpeed
    ) {
        return;
    }

    // AI co the thay doi steer moi frame. Khong reset PI moi lan steer doi,
    // neu khong chu ky 100 ms se khong bao gio duoc chay.
    bool mustResetController = currentMode != 'V';

    requestedSteer = steer;
    requestedSpeed = speed;
    currentMode = 'V';

    int speedMagnitude = abs(speed);
    int baseTarget = speedToBaseTarget(speedMagnitude);

    // steer duong: banh trai nhanh hon, banh phai cham hon -> re phai.
    //
    // 2026-09-18: LAM TRON thay vi cat. O BASE_SPEED=16 (baseTarget=25) mot
    // don vi steer chi dang 0.875 tick, nen phep chia nguyen cat xuong bien
    // ba gia tri steer lien tiep thanh cung mot chenh lech banh. Bo dieu
    // khien be 1, roi 2, roi 3 ma banh khong doi gi, den khi nhay mot cuc -
    // do la co che sinh ra kieu "chay thang roi nhich zigzag". Lam tron
    // khong doi do loi tong, chi tra lai mot nac phan giai.
    long numerator =
        (long)baseTarget *
        (long)steer *
        (long)STEER_GAIN_PERCENT *
        (long)STEER_SIGN;
    long denominator = (long)STEER_COMMAND_MAX * 100L;
    long turnTarget =
        (numerator >= 0)
            ? (numerator + denominator / 2) / denominator
            : -((-numerator + denominator / 2) / denominator);

    // Bu lech co khi NGAY TAI DAY thay vi cong vao lenh steer o Python.
    // Cong vao steer thi no day ca dai lam viec cua PID di, nen mang phang
    // do lam tron khong con nam o tam (0) ma lech han sang mot ben: do duoc
    // steer +1/+2/+3 cho ra y het nhau trong khi -1/-2/-3 thi moi nac mot
    // khac. Dat o day thi trim la offset tick doc lap, mang phang ve dung
    // tam, va hai chieu be co cung do phan giai.
    // -1 tick moi ben = chenh lech -2 tick, dung bang STEER_TRIM=-2 cu.
    leftTargetTicks = constrain(
        baseTarget + (int)turnTarget + STEER_TRIM_TICKS,
        0,
        MAX_WHEEL_TARGET_TICKS
    );

    rightTargetTicks = constrain(
        baseTarget - (int)turnTarget - STEER_TRIM_TICKS,
        0,
        MAX_WHEEL_TARGET_TICKS
    );

    leftDirection  = FORWARD_MOTOR_DIRECTION;
    rightDirection = FORWARD_MOTOR_DIRECTION;

    if (mustResetController) {
        resetController();
    } else {
        // Cap nhat target ma giu nguyen bo dem va integral PI.
        if (leftTargetTicks <= 0) {
            leftPWM = 0;
            leftIntegral = 0.0f;
        } else if (leftPWM == 0) {
            leftPWM = feedForwardPWM(leftTargetTicks);
        }

        if (rightTargetTicks <= 0) {
            rightPWM = 0;
            rightIntegral = 0.0f;
        } else if (rightPWM == 0) {
            rightPWM = feedForwardPWM(rightTargetTicks);
        }
    }

    applyMotorOutputs();
}

// ================= SERIAL PARSER =================

void clearPendingSerialInput()
{
    serialIndex = 0;
    serialDiscarding = false;

    while (Serial.available() > 0) {
        Serial.read();
    }
}

void rejectSerialCommand(const char *message)
{
    stopMotors();
    clearPendingSerialInput();
    Serial.println(message);
}

void processSerialLine(char *line)
{
    int steer = 0;
    int speed = 0;
    char extra = '\0';

    int fields = sscanf(line, "%d %d %c", &steer, &speed, &extra);

    if (fields != 2) {
        rejectSerialCommand("ERR,FORMAT,STOP");
        return;
    }

    if (
        steer < -STEER_COMMAND_MAX ||
        steer > STEER_COMMAND_MAX ||
        speed < 0 ||
        speed > SPEED_COMMAND_MAX
    ) {
        rejectSerialCommand("ERR,RANGE,STOP");
        return;
    }

    startVelocityCommand(steer, speed);
}

void readSerialCommands()
{
    while (Serial.available() > 0) {
        char value = Serial.read();

        if (serialDiscarding) {
            if (value == '\n' || value == '\r') {
                serialDiscarding = false;
                serialIndex = 0;
            }
            continue;
        }

        if (value == '\n' || value == '\r') {
            if (serialIndex > 0) {
                serialBuffer[serialIndex] = '\0';
                processSerialLine(serialBuffer);
                serialIndex = 0;
            }
            continue;
        }

        if (serialIndex < sizeof(serialBuffer) - 1) {
            serialBuffer[serialIndex++] = value;
        } else {
            stopMotors();
            serialIndex = 0;
            serialDiscarding = true;
            Serial.println("ERR,BUFFER_OVERFLOW,STOP");
        }
    }
}

// ================= PI UPDATE =================

void updatePIController()
{
    unsigned long now = millis();

    if (currentMode == 'S') {
        return;
    }

    if (now - lastControlTime < CONTROL_INTERVAL_MS) {
        return;
    }

    lastControlTime = now;

    noInterrupts();
    long currentLeft  = leftEncoderCount;
    long currentRight = rightEncoderCount;
    interrupts();

    leftDelta  = currentLeft - previousLeftCount;
    rightDelta = currentRight - previousRightCount;

    previousLeftCount  = currentLeft;
    previousRightCount = currentRight;

    long measuredLeft  = abs(leftDelta);
    long measuredRight = abs(rightDelta);

    leftPWM = updateOneWheelPI(
        leftTargetTicks,
        measuredLeft,
        leftIntegral
    );

    rightPWM = updateOneWheelPI(
        rightTargetTicks,
        measuredRight,
        rightIntegral
    );
    // FIX 2026-09-04: gioi han duoi la PWM_MIN o day tao ra bat doi xung
    // that su voi banh trai (chi minh updateOneWheelPI() gioi han [0,
    // PWM_MAX], khong co san PWM_MIN) - khi PI muon giam manh (banh phai
    // dang vuot muc tieu), gia tri bi ep cung ve PWM_MIN=70 thay vi duoc
    // giam that, lam banh phai "khong the giam toc". Doi ve [0, PWM_MAX]
    // cho khop cach banh trai dang hoat dong.
    if (rightPWM > 0) {
        rightPWM = constrain(
            (int)(rightPWM * RIGHT_WHEEL_PWM_TRIM + 0.5f),
            0, PWM_MAX
        );
    }

    applyMotorOutputs();
}

// ================= TELEMETRY =================

void printTelemetry()
{
    noInterrupts();
    long leftCount  = leftEncoderCount;
    long rightCount = rightEncoderCount;
    interrupts();

    Serial.print("ENC,");
    Serial.print(leftCount);
    Serial.print(",");
    Serial.print(rightCount);

    Serial.print(",DELTA,");
    Serial.print(leftDelta);
    Serial.print(",");
    Serial.print(rightDelta);

    Serial.print(",TARGET,");
    Serial.print(leftTargetTicks);
    Serial.print(",");
    Serial.print(rightTargetTicks);

    Serial.print(",PWM,");
    Serial.print(leftPWM);
    Serial.print(",");
    Serial.print(rightPWM);

    Serial.print(",STEER,");
    Serial.print(requestedSteer);

    Serial.print(",SPEED,");
    Serial.print(requestedSpeed);

    Serial.print(",MODE,");
    Serial.print(currentMode);

    Serial.print(",SEQ,");
    Serial.print(telemetrySeq);
    telemetrySeq++;

    // IMU: chi doc/bao cao, khong tham gia dieu khien dong co. Neu IMU
    // khong san sang thi in NA de Jetson biet ma khong bi lech cot.
    if (imuReady) {
        sensors_event_t orientationEvent;
        sensors_event_t linAccelEvent;
        bno.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&linAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);

        // 2026-09-12: them toc do quay tho tu con quay hoi chuyen.
        //
        // Dung getVector chu khong getEvent: getVector tra do/giay (1 dps
        // = 16 LSB) con getEvent doi sang rad/giay. Do phan giai 0.0625
        // do/giay - tot hon 16 lan so voi lay vi phan yaw Euler (0.1 do o
        // 20 Hz cho buoc 2 do/giay), va do la ly do truong nay dang gia.
        //
        // In ca ba truc, KHONG chon san truc nao la "toc do quay thang
        // dung". Truc nao ung voi yaw phu thuoc vao cach gan cam bien
        // tren xe, va viec doan sai thi khong the phat hien duoc tu log.
        // Phan tich se tu chon truc bang cach doi chieu voi vi phan cua
        // yaw Euler.
        imu::Vector<3> gyro =
            bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

        float ax = linAccelEvent.acceleration.x;
        float ay = linAccelEvent.acceleration.y;
        float az = linAccelEvent.acceleration.z;
        float accelMag = sqrt(ax * ax + ay * ay + az * az);

        Serial.print(",IMU,");
        Serial.print(orientationEvent.orientation.y, 1);  // roll
        Serial.print(",");
        Serial.print(orientationEvent.orientation.z, 1);  // pitch
        Serial.print(",");
        Serial.print(accelMag, 2);
        Serial.print(",");
        // 2026-09-04: them yaw (huong xoay thuc te cua xe, 0-360 do) -
        // .x trong VECTOR_EULER cua Adafruit BNO055. Chi de BAO CAO qua
        // telemetry them, van CHUA tham gia dieu khien dong co/watchdog.
        Serial.print(orientationEvent.orientation.x, 1);  // yaw
        Serial.print(",");
        Serial.print(gyro.x(), 2);                        // do/s
        Serial.print(",");
        Serial.print(gyro.y(), 2);                        // do/s
        Serial.print(",");
        Serial.print(gyro.z(), 2);                        // do/s
    } else {
        Serial.print(",IMU,NA,NA,NA,NA,NA,NA,NA");
    }

    Serial.println();
}

// ================= SETUP / LOOP =================

void setup()
{
    pinMode(LEFT_RPWM, OUTPUT);
    pinMode(LEFT_LPWM, OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);

    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENCODER_A),
        leftEncoderISR,
        RISING
    );

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_A),
        rightEncoderISR,
        RISING
    );

    Serial.begin(115200);
    stopMotors();

    lastCommandTime = millis();
    lastControlTime = millis();
    lastPrintTime = millis();

    // IMU la tinh nang phu (chi bao cao), KHONG duoc phep chan/lam cham
    // khoi dong dong co neu khong tim thay hoac loi.
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    Wire.begin();
    imuReady = bno.begin();
    Serial.println(imuReady ? "IMU,OK" : "IMU,NOT_FOUND");

    Serial.println("READY,AGV_STM32_SAFE_V2");
}

void loop()
{
    readSerialCommands();

    if (
        currentMode != 'S' &&
        millis() - lastCommandTime > TIMEOUT_MS
    ) {
        stopMotors();
        clearPendingSerialInput();
        Serial.println("FAULT,TIMEOUT");
    }

    updatePIController();

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();
        printTelemetry();
    }
}
