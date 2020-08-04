#include <WifiEspNow.h>
#include "gamma.h"
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#endif
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h>
#define OUTPUT_READABLE_ACCELGYRO
// FLIGHT SYSTEMS THRESHOLDS (to be continued;)
#define MAX_TILT 8192 //45deg
#define MIN_TILT 8192//-45Deg
#define STALL_WARN_THRESHOLD 5461 //30deg
#define FALL_WARN_THRESHOLD -5461//30deg

//JOYSTICKS
int X = 0;
int Y = 0;
int X2 = 128;
int Y2 = 128;
int X2_ap = 128;
int Y2_ap = 128;
int Calib = 0;
// FLIGHT PARAMETERS
int T = 512; // THROTTLE
// MOTORS
int mot_L = 0;
int mot_R = 0;
int CALIB_L = 0;
int CALIB_R = 0;
bool mot_power = false;
bool connection = false;
bool motors = false;
bool TelemetrySend = false;
//  HEIGHT STEERING
int Serv_1_MAX = 140;
int Serv_1_MIN = 30;
int Serv_1_DEF = 90;
int ServL_VAL = 90;
int ServR_VAL = 90;
int ServL_CAL = 5;//85
int ServR_CAL = -10;//80
int Serv_DIFF = 0;
// REFRESHING
int ServInterval = 100;
int C = 0;
/// SYSTEMS
bool AFS_en = 1;
bool ASS_en = 1;
bool AP_en = 0;
bool TH_en = 0;
bool AFS_act = 0;
bool ASS_act = 0;
bool AP_act = 0;
bool TH_act = 0;
bool full_telemetry = false;
bool gyro_telemetry = false;
// Gyroscope Init
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long previousMillis = 0;
unsigned long messageTime = 0;
unsigned long FlightTime = 0;
// REMOTE CLIENT MAC address:
static uint8_t PEER[] {0xA4, 0xCF, 0x12, 0x25, 0x66, 0x85};
Servo ServL;
Servo ServR;
#include "motors.h"

#define REFRESH_MULTIPLIER 1
#define LIGHT_PIN D4
ADC_MODE(ADC_VCC);
// flasher class created to initialize signal light
Flasher Pos_LED(LIGHT_PIN, 100, 2000);
Flasher Alert_LED(LIGHT_PIN, 50, 400);

// Printing telemetry thru serial port.
void SerialTelemetryData() {

  if (full_telemetry) {
    Serial.print(mot_L);
    Serial.print(",");
    Serial.print(mot_R);
    Serial.print(",");
    Serial.print(X);
    Serial.print(",");
    Serial.print(Y);
    Serial.print(",");
    Serial.print(X2);
    Serial.print(",");
    Serial.print(Y2);
    Serial.print(",");

    Serial.print(T);
    Serial.print(",");
    Serial.print(ServL_VAL);
    Serial.print(",");
    Serial.print(ServR_VAL);
    Serial.print(",");
    Serial.print(Serv_DIFF);
    Serial.print(",");
    Serial.print(ESP.getVcc());
    Serial.print(",");
  };

  // GYRO
  if (gyro_telemetry) {
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.print(",");
  Serial.print(sqrt(pow(az, 2) + pow(ax, 2) + pow(ay, 2)));
  Serial.print(",");
  Serial.print(sqrt(pow(gz, 2) + pow(gx, 2) + pow(gy, 2)));
  Serial.println("");
  }
  //  //Serial.print("gX:"); Serial.print(gx);
  //  Serial.print(" gXm:"); Serial.print(map(gx, -32768, 32768, -180, 180)); Serial.print("\t");
  //  //Serial.print("gY:"); Serial.print(gy);
  //  Serial.print(" gYm:"); Serial.print(map(gy, -32768, 32768, -180, 180)); Serial.print("\t");
  //  //Serial.print("gZ:"); Serial.print(gz);
  //  Serial.print(" gZm:"); Serial.print(map(gz, -32768, 32768, -180, 180)); Serial.println("\t");
}

void TelemetrySendMessage () {
  uint8_t msg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  // Protocol description
  // 0 - Status switches in 8bit binary format!
  // 1 - mot_l - MOTOR L Power
  // 2 - mot_r - MOTOR R Power
  // 3 - bat_lev 0-4V - Battery level - currently unsupported
  // 4 - Serv L - left servo value
  // 5 - Serv R - right servo value
  // END OF ACTUAL TELEMETRY
  // 6 - Tilt Y
  // 7 - Tilt Z
  //8 - Tilst X Y Z
  //
  //
  //
  //
  bitWrite(msg[0], 0, mot_power);
  bitWrite(msg[0], 1, AFS_act);
  bitWrite(msg[0], 2, ASS_act);
  bitWrite(msg[0], 3, AP_act);
  bitWrite(msg[0], 4, TH_act);

  msg[1] = map(mot_L, 0, 1023, 0, 255);
  msg[2] = map(mot_R, 0, 1023, 0, 255);
  msg[3] = map(ESP.getVcc(), 0, 4096, 0, 255);
  msg[4] = ServL_VAL;
  msg[5] = ServR_VAL;
  msg[6] = map(ax , -16384 , 16384 , 0, 255);
  msg[7] = map(ay , -16384 , 16384 , 0, 255);
// Send telemetry thru ESP NOW:
  WifiEspNow.send(PEER, msg, sizeof(msg));
}
// recieving control data
void printReceivedMessage(const uint8_t mac[6], const uint8_t* buf, size_t count, void* cbarg) {

// MAIN MOTOR POWER SWITCH
  if (bitRead(buf[0], 0)) {
    mot_power = true;
  }  else {
    mot_power = false;
  };

// MAIN JOYSTICK X and Y VALUES ARE ENCODED IN 2 8bit values (1-4) to extend precision from 256 to 512 levels
  Y = buf[1] - buf[2];
  X = buf[3] - buf[4];
  // MOTORS CALIBRATION VALUE 0-255
  Calib = buf[5];
  if (Calib > 128) {
    CALIB_L = map(Calib, 128, 255, 0, 255);
    CALIB_R = 0;
  }
  else if (Calib < 128) {
    CALIB_R = map(Calib, 128, 255, 0, 255);
    CALIB_L = 0;
  };
  // THROTTLE VALUE BY default throttle is set from 0-1024 but in 8bit precision
  T = buf[6] * 2 + 2;
  if (T > 1023) {
    T = 1023;

  };
  // 7th value is an communications refresh interval. When it is 0 telemetry request is sent from remote
  if (buf[7] > 0) {
    ServInterval = buf[7] * REFRESH_MULTIPLIER;
  } else if (buf[7] == 0)
  {
    TelemetrySendMessage();
  }
  messageTime = millis();
  // second joystick is 8 bit precision 
  X2 = buf[8];
  Y2 = buf[9];
// onboard safety systems 
  AFS_en = bitRead(buf[0], 1); // ANTI FALL SYSTEM
  ASS_en = bitRead(buf[0], 2); // ANTI STALL SYSTEM
  AP_en = bitRead(buf[0], 3); // AUTO PILOT - enables joystick to approach desired angle of flight
  TH_en = bitRead(buf[0], 4); // TILT HELPER - helps to maintain proper level flight when remote control is offline
}

void setup() {
  // INITIAL SETUP
  Serial.begin(115200);
  Serial.println();
  // ENGINES INITIALIZE
  digitalWrite(MOT_L_PIN, 0);
  digitalWrite(MOT_R_PIN, 0);
  // GYROSCOPE

  Serial.println("Initializing I2C devices...");
  Wire.begin(D1, D2);
  // MPU6050
  accelgyro.initialize();
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values

  Serial.println("Updating internal sensor offsets...");
  // -76  -2359 1688  0 0 0
  //  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  //  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  //  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  //  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  //  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  //  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  //  Serial.print("\n");


  // OFFSET VALUES HAS TO BE CHANGED AND DETERMINED TO LEVEL FLIGHT
  accelgyro.setXGyroOffset(230);
  accelgyro.setYGyroOffset(-27);
  accelgyro.setZGyroOffset(37);
  accelgyro.setXAccelOffset(433);
  accelgyro.setYAccelOffset(-3337);
  accelgyro.setZAccelOffset(4739);
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");

  // TESTING SERVOES
  ServL.attach(D5);
  ServR.attach(D6);
  delay(500);
  ServL.write(Serv_1_MIN);
  ServR.write(Serv_1_MAX);
  delay(500);
  ServL.write(Serv_1_MAX);
  ServR.write(Serv_1_MIN);
  delay(500);
  ServL.write(Serv_1_DEF + ServL_CAL);
  ServR.write(Serv_1_DEF + ServR_CAL);
  delay(500);
  /// COMMUNICATION CONFIG
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESPNOW", nullptr, 3);
  WiFi.softAPdisconnect(false);

  Serial.print("MAC address of this node is ");
  Serial.println(WiFi.softAPmacAddress());

  bool ok = WifiEspNow.begin();
  if (!ok) {
    Serial.println("WifiEspNow.begin() failed");
    ESP.restart();
  }
  // COMMUNICATION INITIALISATION
  WifiEspNow.onReceive(printReceivedMessage, nullptr);

  ok = WifiEspNow.addPeer(PEER);
  if (!ok) {
    Serial.println("WifiEspNow.addPeer() failed");
    ESP.restart();

  };
  Serial.println("---------------DATA EXPORT----------------");
  //    Serial.println("");
  if (full_telemetry) {
    Serial.print("ML, MR,jX,jY,j2X,j2Y,Th,SvoL,SvoR,SvoDiff,A0,");
  }
  // GYROS
  if (gyro_telemetry) {
  Serial.println("ax,ay,gx,gy");
}
}
void loop() {

  unsigned long currentMillis = millis();
  if (millis() - messageTime > 15 * ServInterval) {
    // WHEN DISCONNECTED ---------------------------------------------------------------------// timeout condition

    connection = false;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //   Serial.println("No Connection Zeroing Controls!");
    X = 0;
    Y = 0;
    if (AP_en) {
      AP_act = true;
      if (ax >= 0) {
        X2_ap = X2 - map(ax, -32768 / 4, 32767 / 4, 0, 128);
      } else if (ax < 0) {
        X2_ap = X2 + map(-ax, -32768 / 4, 32767 / 4, 0, 128);
      };;
      // tu mozna sprobowac zrobic autopilota (level flight)
      Servoes(X2_ap, Y2_ap);
    } else {
      Servoes(X2, Y2);
      AP_act = false;
    };

    mot_power = false;
    //   Serial.println("NO CONN");
    T = 0;
    //    B = 0;
    Motors(mot_power , X, Y, T);

  } else if (currentMillis - previousMillis >= ServInterval) {
    // WHEN CONNECTED ---------------------------------------------------------------------
    connection = true;

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    SerialTelemetryData();
    previousMillis = currentMillis;
    AP_act = false;
    // ANTI FALL SYSTEM
    if (AFS_en && ax < FALL_WARN_THRESHOLD) {
      AFS_act = true;
      Motors(mot_power, X, -255, T);
      Servoes(255, Y2);
      //  Serial.println("FAll WARNING");

      //ANTI STALL STALL SYSTEM
    } else if (ASS_en && ax > STALL_WARN_THRESHOLD) {
      ASS_act = true;
      Motors(mot_power, X, 255, T);
      Servoes(0, Y2);
      //  Serial.println("STALL WARNING");
    } else {

      AFS_act = false;
      ASS_act = false;
      // TILT HELPER SYSTEM
      if (TH_en)
      {
#define MAX_TH_TILT -32768/6 //30 degreees
        int X_diff = map(X2, 255, 0, MAX_TH_TILT * -1, MAX_TH_TILT ) - ax;
        int T_diff = map(X2, 0, 255, MAX_TH_TILT * -1, MAX_TH_TILT ) - abs(ax);
        int T_th = 512 + map(T_diff, MAX_TH_TILT * -1, MAX_TH_TILT , -512, 512);
        int X2_th = map(X_diff * -1, MAX_TH_TILT * -1, MAX_TH_TILT, 0, 255);
        // warunki skrajne:
        if (T_th > 1023) {
          T_th = 1023;
        };
        if (T_th < 0) {
          T_th = 0;
        };
        if (X2_th > 255) {
          X2_th = 255;
        };
        if (X2_th < 0) {
          X2_th = 0;
        };
        //       Serial.println(X_diff);// +-15 degrees of tilt
        Motors(mot_power, X, Y, T_th);
        Servoes(X2_th , Y2);

      }
      else {
        AFS_act = false;
        ASS_act = false;
        Motors(mot_power, X, Y, T);
        Servoes(X2, Y2);
      }
    };


    // WHEN CONNECTED ---------------------------------------------------------------------
  };



  // OUT OF MAIN REFRESH INTERVAL-------------------------------------------------------------------


  if (connection) {
    // OTHER WHEN MOTORS ON ---------------------------------------------------------------------
    Pos_LED.Update();
    // Alt1.Update();
  }
  else {
    // OTHER WHEN MOTORS OOFF ---------------------------------------------------------------------
    Alert_LED.Update();

  };


  //delay(100);
}
