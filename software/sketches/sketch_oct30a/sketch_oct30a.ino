/*
 * 四足歩行ロボット 歩行プログラム（クロール歩行）
 * Quadruped robot walking program (crawl gait)
 *
 *  - マイコン / MCU   : ESP32 (WROOM-32E)
 *  - サーボ / Servos  : 1脚につき2個 × 4脚 = 8個 / 2 per leg x 4 legs = 8
 *      HIP  : 脚を前後に振る（水平回転、時計/反時計回り）
 *             Swings the leg forward/backward (horizontal rotation, CW/CCW)
 *      LIFT : 脚を上げ下げする / Raises and lowers the leg
 *  - 脚の順番 / Leg order : 右後 → 右前 → 左後 → 左前
 *                           Rear-right -> Front-right -> Rear-left -> Front-left
 *
 * 必要なライブラリ（ライブラリマネージャから追加）
 * Required libraries (install from the Library Manager)
 *  - 直接接続 / Direct connection : ESP32Servo
 *  - PCA9685使用 / Using PCA9685  : Adafruit PWM Servo Driver Library
 */

// ================================================================
//  設定（ここを自分の機体に合わせて変更する）
//  Settings (adjust these to match your robot)
// ================================================================

// 0 : ESP32のピンにサーボを直接つなぐ / Servos connected directly to ESP32 pins
// 1 : PCA9685サーボドライバ経由でつなぐ / Servos connected via a PCA9685 driver
#define USE_PCA9685 0

// 1にすると全サーボを中立位置で止める（組み立て・調整用）
// Set to 1 to hold all servos at neutral (for assembly and calibration)
#define CALIBRATION_MODE 0

// 脚の番号 / Leg indices
enum Leg { RF = 0, LF = 1, RR = 2, LR = 3 };
const char* LEG_NAME[4] = {"FrontRight", "FrontLeft", "RearRight", "RearLeft"};

// 歩く順番：右後 → 右前 → 左後 → 左前
// Step order: rear-right -> front-right -> rear-left -> front-left
const int ORDER[4] = {RR, RF, LR, LF};

// --- 直接接続する場合のピン番号 / Pin numbers for direct connection ---
//                           RF   LF   RR   LR
const uint8_t HIP_PIN[4]  = {13,  27,  25,  32};
const uint8_t LIFT_PIN[4] = {14,  26,  33,  16};

// --- PCA9685を使う場合のチャンネル番号 / PCA9685 channel numbers ---
const uint8_t HIP_CH[4]  = {0, 2, 4, 6};
const uint8_t LIFT_CH[4] = {1, 3, 5, 7};

// --- サーボのパルス幅（使うサーボの仕様に合わせる） ---
// --- Servo pulse width (match your servo's spec) ---
const int SERVO_MIN_US = 500;   // 0度のパルス幅 / Pulse width at 0 deg
const int SERVO_MAX_US = 2500;  // 180度のパルス幅 / Pulse width at 180 deg

// --- 組み付け誤差の補正（度） ---
// CALIBRATION_MODEで脚がまっすぐになるよう調整する
// --- Trim for assembly error (degrees) ---
// Adjust in CALIBRATION_MODE until each leg is straight
int HIP_TRIM[4]  = {0, 0, 0, 0};
int LIFT_TRIM[4] = {0, 0, 0, 0};

// --- 回転方向（+1 か -1） / Rotation direction (+1 or -1) ---
// HIP  : プラスの角度で脚が「前」に振れるように合わせる
//        Set so that a positive angle swings the leg FORWARD
// LIFT : プラスの角度で脚が「上」に上がるように合わせる
//        Set so that a positive angle raises the leg UP
// 左右でサーボが鏡写しに付いていることが多いので、初期値は左を-1にしている
// Left and right servos are usually mirrored, so left side defaults to -1
const int HIP_DIR[4]  = {+1, -1, +1, -1};
const int LIFT_DIR[4] = {+1, -1, +1, -1};

// --- 歩行パラメータ / Gait parameters ---
const float STRIDE  = 30.0;  // 1歩で脚を前後に振る角度（度）/ Swing angle per step (deg)
const float LIFT_UP = 30.0;  // 脚を上げる角度（度）/ Lift angle (deg)
const float STAND   = 0.0;   // 接地時のLIFTの角度（度）/ LIFT angle when on the ground (deg)

// 各動作にかける時間（ミリ秒）。転びやすいときはゆっくりにする
// Duration of each motion (ms). Slow down if the robot tips over
const int T_LIFT  = 150;  // 脚を上げる / Lift the leg
const int T_SWING = 200;  // 脚を前に出す / Swing the leg forward
const int T_LOWER = 150;  // 脚を下ろす / Lower the leg
const int T_PUSH  = 300;  // 胴体を前に送る / Push the body forward
const int STEP_MS = 10;   // 補間の刻み / Interpolation interval

// ================================================================
//  サーボの出力 / Servo output
// ================================================================

#if USE_PCA9685
  #include <Wire.h>
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
#else
  #include <ESP32Servo.h>
  Servo hipServo[4];
  Servo liftServo[4];
#endif

// 現在の角度（中立からの差。HIPは前がプラス、LIFTは上がプラス）
// Current angles (offset from neutral; HIP: forward is +, LIFT: up is +)
float hipNow[4];
float liftNow[4];

// 論理角度（中立からの差）を実際のサーボ角に変換して出力する
// Convert a logical angle (offset from neutral) to a real servo angle and output it
void writeServo(bool isHip, int leg, float logicalDeg) {
  int dir  = isHip ? HIP_DIR[leg]  : LIFT_DIR[leg];
  int trim = isHip ? HIP_TRIM[leg] : LIFT_TRIM[leg];

  float deg = 90.0 + trim + dir * logicalDeg;
  deg = constrain(deg, 0.0, 180.0);
  int us = SERVO_MIN_US + (int)((SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0);

#if USE_PCA9685
  pca.writeMicroseconds(isHip ? HIP_CH[leg] : LIFT_CH[leg], us);
#else
  if (isHip) hipServo[leg].writeMicroseconds(us);
  else       liftServo[leg].writeMicroseconds(us);
#endif
}

// 全サーボに現在の角度を出力する / Write current angles to all servos
void applyAll() {
  for (int i = 0; i < 4; i++) {
    writeServo(true,  i, hipNow[i]);
    writeServo(false, i, liftNow[i]);
  }
}

// 全サーボを目標角度まで、指定時間かけてなめらかに動かす
// Move all servos smoothly to the target angles over the given duration
void moveTo(const float hipTarget[4], const float liftTarget[4], int durationMs) {
  float hipStart[4], liftStart[4];
  for (int i = 0; i < 4; i++) {
    hipStart[i]  = hipNow[i];
    liftStart[i] = liftNow[i];
  }

  int steps = max(1, durationMs / STEP_MS);
  for (int s = 1; s <= steps; s++) {
    float t = (float)s / steps;
    // 始めと終わりをゆっくりにする / Ease in and ease out
    float e = t * t * (3.0 - 2.0 * t);
    for (int i = 0; i < 4; i++) {
      hipNow[i]  = hipStart[i]  + (hipTarget[i]  - hipStart[i])  * e;
      liftNow[i] = liftStart[i] + (liftTarget[i] - liftStart[i]) * e;
    }
    applyAll();
    delay(STEP_MS);
  }
}

// ================================================================
//  歩行 / Walking
// ================================================================

/*
 * 1脚ぶんの動作 / Motion for one leg
 *   1. 脚を上げる / Lift the leg
 *   2. 脚を前に出す（-STRIDE/2 → +STRIDE/2）/ Swing it forward (-STRIDE/2 -> +STRIDE/2)
 *   3. 脚を下ろす / Lower the leg
 *   4. 4脚すべてを STRIDE/4 だけ後ろへ振り、胴体を前に送る
 *      Swing all 4 legs back by STRIDE/4 to push the body forward
 *
 * 4脚が1回ずつ動くと、どの脚も前に STRIDE、後ろに STRIDE/4 × 4 = STRIDE 動くので
 * 1サイクルで元の位置に戻る。
 * After each leg moves once, every leg has moved forward by STRIDE and back by
 * STRIDE/4 x 4 = STRIDE, so all legs return to their starting positions.
 */
void stepLeg(int leg) {
  float h[4], l[4];
  for (int i = 0; i < 4; i++) {
    h[i] = hipNow[i];
    l[i] = liftNow[i];
  }

  // 1. 上げる / Lift
  l[leg] = LIFT_UP;
  moveTo(h, l, T_LIFT);

  // 2. 前に出す / Swing forward
  h[leg] = STRIDE / 2.0;
  moveTo(h, l, T_SWING);

  // 3. 下ろす / Lower
  l[leg] = STAND;
  moveTo(h, l, T_LOWER);

  // 4. 胴体を前に送る / Push the body forward
  for (int i = 0; i < 4; i++) h[i] -= STRIDE / 4.0;
  moveTo(h, l, T_PUSH);
}

// 1サイクル（右後 → 右前 → 左後 → 左前）
// One gait cycle (rear-right -> front-right -> rear-left -> front-left)
void walkCycle() {
  for (int k = 0; k < 4; k++) {
    stepLeg(ORDER[k]);
  }
}

// 初期姿勢：次に動く脚ほど後ろに置き、脚の位置をずらしておく
// Initial stance: legs that move sooner are placed further back (staggered)
//   RR -STRIDE/2, RF -STRIDE/4, LR 0, LF +STRIDE/4
void standPose(int durationMs) {
  float h[4], l[4];
  for (int k = 0; k < 4; k++) {
    h[ORDER[k]] = -STRIDE / 2.0 + k * STRIDE / 4.0;
    l[ORDER[k]] = STAND;
  }
  moveTo(h, l, durationMs);
}

// ================================================================
//  setup / loop
// ================================================================

bool walking = false;

void setup() {
  Serial.begin(115200);

#if USE_PCA9685
  Wire.begin();
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);
#else
  for (int i = 0; i < 4; i++) {
    hipServo[i].setPeriodHertz(50);
    liftServo[i].setPeriodHertz(50);
    hipServo[i].attach(HIP_PIN[i],  SERVO_MIN_US, SERVO_MAX_US);
    liftServo[i].attach(LIFT_PIN[i], SERVO_MIN_US, SERVO_MAX_US);
  }
#endif

  // まず全サーボを中立にする / First, set all servos to neutral
  for (int i = 0; i < 4; i++) {
    hipNow[i]  = 0;
    liftNow[i] = STAND;
  }
  applyAll();
  delay(1000);

#if CALIBRATION_MODE
  Serial.println("Calibration mode: all servos at neutral. Adjust TRIM values.");
#else
  standPose(800);
  Serial.println("Ready. Send 's' in the Serial Monitor to start/stop walking.");
#endif
}

void loop() {
#if CALIBRATION_MODE
  return;  // 中立のまま何もしない / Stay at neutral, do nothing
#else
  // 's' で歩行の開始／停止を切り替える / Toggle walking with 's'
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') {
      walking = !walking;
      Serial.println(walking ? "Walking" : "Stopped");
      if (!walking) standPose(500);
    }
  }

  if (walking) {
    walkCycle();
  }
#endif
}