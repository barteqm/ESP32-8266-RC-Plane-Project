#include "ownJoystick.h"

#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
SSD1306  display(0x3c, 17, 16);
#include <WifiEspNow.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#endif
#define INTERVAL 70
#define SCREEN_INTERVAL 100
#define EN_PIN 22
#define STARTER_PIN 19

uint8_t mot_power = 0;
uint8_t screen_page = 0;
int ScreenSaverTimer = 0;
#define SCREENSAVER_TIME 18000 // two minutes
int T = 255;
int Calib = 128;
int Lev = 128;
static uint8_t PEER[6] {0xA2, 0x20, 0xA6, 0x14, 0x3D, 0xF8}; //d1 mini dlugie nozki
// TELEMETRY DEFINITIONS
int TelemetryCounter = 0;
int TelemetryErrorCounter = 0;
int TelemetryThreshold = 10;
int TelemetryMultiplier = 10; //Sending telemetry request every 10 signals
uint8_t t_power = 0;
uint8_t t_mot_L = 0;
uint8_t t_mot_R = 0;
uint8_t t_b_lev = 0 ;
uint8_t t_Serv_L = 0;
uint8_t t_Serv_R = 0;
uint8_t t_tilt_X = 0;
uint8_t t_tilt_Y = 0;
bool AFS_act = false;
bool ASS_act = false;
bool AP_act = false;
bool TH_act = false;

// starter mod
bool starter_en = false;
int starter_timer = 0;
#define STARTER_MULTIPLIER 180
// jpystickie
MyJoystick Joy1(32, 33, 27);
MyJoystick Joy2(34, 35, 26);
// enable button 22
// 18 - scl oled, 19 - sda oled

// JOYSTICK PARAMETERS
bool AFS_en = 1;
bool ASS_en = 0;
bool AP_en = 1;
bool TH_en = 0;
bool servoes = true;
uint8_t msg[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//                0  1  2  3  4  5  6  7  8   9
// 0 - motors enable /modes switch
//
// 1 - Joy1 X +
// 2 - Joy1 X -
// 3 - Joy1 Y +
// 4 - Joy1 Y -
// 5 - Calib
// 6 - THROTTLE
// 7 - INTERVAL sync /telemetry send (when 0)
// 8 - Joy2 X
// 9 - Joy2 Y


//
//
//
//
int X = 0;
int Y = 0;
int X2 = 0;
int Y2 = 0;
int B = 0;
int DataCounter;
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;

void printReceivedMessage(const uint8_t mac[6], const uint8_t* buf, size_t count, void* cbarg) {
  Serial.print("Telemetry Data: ");
  for (int i = 0; i < count; ++i) {
    Serial.print((buf[i]));
    Serial.print(",");

  }
  t_power = buf[0];
  t_mot_L =  buf[1];
  t_mot_R =  buf[2];
  t_b_lev =  buf[3] ;
  t_Serv_L = buf[4];
  t_Serv_R = buf[5];
  t_tilt_X = buf[6];
  t_tilt_Y = buf[7];
  AFS_act = bitRead(buf[0], 1);
  ASS_act = bitRead(buf[0], 2);
  AP_act = bitRead(buf[0], 3);
  TH_act = bitRead(buf[0], 4);
  TelemetryErrorCounter = 0;
  Serial.println();
}
void WriteHorizontalDottedLine(int X1, int Y1, int L) {
  for (int i = X1; i < L; i = i + 3) {
    display.fillRect(i, Y1, 1, 1);
  };
}
void WriteVerticalDottedLine(int X1, int Y1, int L) {
  for (int i = Y1; i < L; i = i + 3) {
    display.fillRect(X1, i, 1, 1);
  };
}
void DispMainScreen() {
  display.drawLine(0, 16, 128, 16); //poz

  display.drawLine(0, 63, 128, 63); //poz
  WriteHorizontalDottedLine(0, 39, 94);
  WriteVerticalDottedLine(24, 17, 60);
  WriteVerticalDottedLine(24 + 46, 17, 60);
  display.drawLine(25, 3, 108, 3);
  display.drawLine(46, 3, 46, 16);
  display.drawLine(66, 3, 66, 16);
  display.drawLine(86, 3, 86, 16);
  //    display.drawLine(46, 3, 46, 16);
  display.drawLine(25, 0, 25, 16); //pion bat
  display.drawLine(108, 0, 108, 16); //pion EN
  display.drawLine(0, 16, 0, 64); // Joy.l 102 110 111 118
  display.drawLine(47, 16, 47, 64); // Joy.l 102 110 111 118

  display.drawLine(94, 16, 94, 64); // Joy.l
  display.drawLine(102, 16, 102, 64); // Joy.l
  display.drawLine(110, 16, 110, 64); // Joy.l
  display.drawLine(111, 16, 111, 64); // Joy.l
  display.drawLine(119, 16, 119, 64); // Joy.l
  display.drawLine(127, 16, 127, 64); // Joy.l
  display.setFont(ArialMT_Plain_10);
  // display.drawString(0, 2, "BAT");
  display.drawString(2, 17, "JL");

  display.drawString(48, 17, "JR");

}
void DispMainScreenData() {


  if (TelemetryErrorCounter > TelemetryThreshold && !starter_en) {
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 2, "ERR");
  } else {
    //BATTERY
    if (starter_en) {
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 2, "STA");
    } else {
      display.drawString(0, 2, "OK!");
    };
    display.fillRect(25, 0, map(t_b_lev, 0, 255, 0, 83), 2);
    // motor L
    display.fillRect(95, 17, 7, map(t_mot_L, 0, 255, 0, 45 ));

    display.fillRect(103, 17, 7, map(t_mot_R, 0, 255, 0, 45 ));

    display.fillRect(112, 17, 7, map(t_Serv_L, 0, 180, 0, 45 ));

    display.fillRect(120, 17, 7, map(t_Serv_R, 0, 180, 0, 45 ));
  };
  if (AFS_en) {
    display.drawString(28, 5, "AF");
  }
  if (AFS_act) {
    //  display.drawString(28, 5, "AF");
    display.drawRect(27, 5, 17, 9);

  };
  if (ASS_en) {
    display.drawString(49, 5, "AS");

  };
  if (ASS_act) {
    //  display.drawString(28, 5, "AF");
    display.drawRect(48, 5, 17, 10);

  };
  if (AP_en) {
    display.drawString(69, 5, "AP");
  };
  if (AP_act) {
    //  display.drawString(28, 5, "AF");
    display.drawRect(68, 5, 17, 10);

  };
  if (TH_en) {
    display.drawString(89, 5, "TH");
  };
  if (TH_act) {
    //  display.drawString(28, 5, "AF");
    display.drawRect(90, 5, 17, 10);
  }

  // joysticki
  display.fillRect(1 + map(Y, -255, 255, 0, 45), 16 + map(X * -1, -255, 255, 0, 45), 3, 3);

  display.fillRect(47 + map(Y2, -255, 255, 0, 45),  16 + map(X2 * -1, -255, 255, 0, 45), 3, 3);

  ///DODATKOWO
  if (bitRead(mot_power, 0)) {
    display.setFont(ArialMT_Plain_10);
    display.drawString(111, 2, "EN");

  } ;

  if (TelemetryCounter > TelemetryMultiplier) {
    display.setFont(ArialMT_Plain_10);
    display.drawString(111, 2, "TR");
  }
  if (t_power == 1) {
    display.setFont(ArialMT_Plain_10);
    display.drawRect(110, 0, 17, 16);

  }

}

void DispHorizon() {
  //display.drawLine(0, 32, 128, 32);
  WriteHorizontalDottedLine(0, 32, 128);
  WriteVerticalDottedLine(64, 0, 64);
  display.drawCircle(64, 32, 15);
  display.drawLine(0, map(t_tilt_X - map(t_tilt_Y, 0, 255, -255, 255), 0, 255, 0, 64) * -1, 128, map(t_tilt_X + map(t_tilt_Y, 0, 255, -255, 255) * -1, 0, 255, 0, 64)); // Joy.l
}
void setup() {
  Serial.begin(115200);
  // WIFI
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESPNOW", nullptr, 3);
  WiFi.softAPdisconnect(false);
  if (display.init()) {
    Serial.println("Display init OK !");

  }
  else {
    Serial.println("Display init Error");
  };
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  //display.clear();
  display.drawString(0, 0, "Joystick v0.002");
  display.display();
  delay(1000);
  Serial.print("MAC address of this node is ");
  Serial.println(WiFi.softAPmacAddress());

  bool ok = WifiEspNow.begin();
  if (!ok) {
    Serial.println("WifiEspNow.begin() failed");
    ESP.restart();
  }
  // ESPNOW ON RECEIVE
  WifiEspNow.onReceive(printReceivedMessage, nullptr);


  ok = WifiEspNow.addPeer(PEER);
  if (!ok) {
    Serial.println("WifiEspNow.addPeer() failed");
    ESP.restart();
  }
  // touchPin
  pinMode(EN_PIN, INPUT);
  pinMode(STARTER_PIN, INPUT_PULLUP);

  // JOYSTICKS
  display.drawString(0, 20, "Calibrating Joy L..");
  display.display();
  Joy1.calibrate(255, 255);
  display.drawString(0, 40, "Calibrating Joy R...");
  display.display();
  Joy2.calibrate(255, 255);
  //  Serv1.attach(25);
  //  Serv2.attach(26);
  display.clear();
  display.drawString(0, 00, "Calibration Finished!");
  display.display();
  delay(500);
  display.clear();
  display.display();
}
void SendData (bool telemetry) {
  //Serial.println("<S>");
  X = Joy1.getX() * -1;
  Y = Joy1.getY() * -1;
  //B = Joy1.getB();
  X2 = Joy2.getX() * -1;
  Y2 = Joy2.getY() * -1;

  if (!starter_en) {
    bitWrite(mot_power, 0, digitalRead(EN_PIN));
    bitWrite(mot_power, 1, AFS_en);
    bitWrite(mot_power, 2, ASS_en);
    bitWrite(mot_power, 3, AP_en);
    bitWrite(mot_power, 4, TH_en);
    bitWrite(mot_power, 5, starter_en);
    // STANDARD PROCEDURE
    msg[0] = mot_power;
    if (X > 0) {
      msg[1] = abs(Joy1.getX());
      msg[2] = 0;
    };
    if (X < 0) {
      msg[2] = abs(Joy1.getX());
      msg[1] = 0;
    };
    if (Y > 0) {
      msg[3] = abs(Joy1.getY());
      msg[4] = 0;
    };
    if (Y < 0) {
      msg[4] = abs(Joy1.getY());
      msg[3] = 0;
    };
    if (X == 0) {
      msg[1] = 0;
      msg[2] = 0;
    };
    if (Y == 0) {
      msg[3] = 0;
      msg[4] = 0;
    };
    msg[5] = Calib;
    msg[6] = T;
    if (telemetry) {
      msg[7] = 0;
    } else {
      msg[7] = INTERVAL;
    };
    msg[8] = map (X2, 255, -255, 0, 255);
    msg[9] = map (Y2, 255, -255, 0, 255);
  }
  else {
    // AUTO START PROCEDURE
    bitWrite(mot_power, 0, 1);
    bitWrite(mot_power, 1, 0);
    bitWrite(mot_power, 2, 0);
    bitWrite(mot_power, 3, 0);
    bitWrite(mot_power, 4, 1);
    bitWrite(mot_power, 5, 1);
    bitWrite(mot_power, 5, starter_en);
    msg[0] = mot_power;
    msg[1] = 255;
    msg[2] = 0;
    msg[3] = 0;
    msg[4] = 0;
    msg[5] = Calib;
    msg[6] = T;
    if (telemetry) {
      msg[7] = 0;
    } else {
      msg[7] = INTERVAL;
    };
    msg[8] = map (X2, 255, -255, 0, 255);
    msg[9] = map (Y2, 255, -255, 0, 255);
  }



  WifiEspNow.send(PEER, msg, sizeof(msg));



}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long currentMillis2 = millis();
  // SCREEEN


  if (currentMillis2 - previousMillis2 >= SCREEN_INTERVAL) {
    ScreenSaverTimer++;
    if (digitalRead(EN_PIN) or Joy1.getB() or Joy2.getB()) {
      ScreenSaverTimer = 0;
    }
    display.clear();
    if (screen_page == 1) {
      display.setFont(ArialMT_Plain_16);
      //    display.clear();
      display.drawString(1, 0, "AntiStall (ASS)");
      if (ASS_en) {
        display.setFont(ArialMT_Plain_24);
        display.drawString(40, 20, "ON");
      } else {
        display.setFont(ArialMT_Plain_24);
        display.drawString(40, 20, "OFF");
      }

    } else if (screen_page == 2) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(1, 0, "AntiFall (AFS)");
      if (AFS_en) {
        display.setFont(ArialMT_Plain_24);
        display.drawString(40, 20, "ON");
      } else {
        display.setFont(ArialMT_Plain_24);
        display.drawString(40, 20, "OFF");
      }
    } else if (screen_page == 3) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(1, 0, "AutoPilot (AP)");
      if (AP_en) {
        display.setFont(ArialMT_Plain_24);
        display.drawString(40, 20, "ON");
      } else {
        display.setFont(ArialMT_Plain_24);
        display.drawString(40, 20, "OFF");
      }
    } else if (screen_page == 4) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(1, 0, "TiltHelper (TH)");
      if (TH_en) {
        display.setFont(ArialMT_Plain_24);
        display.drawString(40, 20, "ON");
      } else {
        display.setFont(ArialMT_Plain_24);
        display.drawString(40, 20, "OFF");
      }
    } else if (screen_page == 5) {
      DispHorizon();
    } else if (screen_page == 9) {
      //Empty PAGE
    } else if (screen_page == 6) {
      display.setFont(ArialMT_Plain_16);
      //    display.clear();
      display.drawString(0, 0, "EN");
      display.drawString(35, 0, "X1");
      display.drawString(70, 0, "Y1");
      display.drawString(100, 0, "X2");
      // output
      display.drawString(0, 20, String(mot_power));
      display.drawString(35, 20, String(X));
      display.drawString(70, 20, String(Y));
      display.drawString(100, 20, String(X2));


    } else if (screen_page == 7) {

      display.clear();
      display.setFont(ArialMT_Plain_16);

      display.drawString(0, 0, "B");
      display.drawString(35, 0, "");
      display.drawString(70, 0, "M1");
      display.drawString(100, 0, "M2");
      // output
      display.drawString(0, 20, String(map(t_b_lev, 0, 255, 0, 4096)));
      display.drawString(35, 20, " ");
      display.drawString(70, 20, String(t_mot_L));
      display.drawString(100, 20, String(t_mot_R));

    } else if (screen_page == 8) {
      //    display.clear();
      display.setFont(ArialMT_Plain_16);
      display.clear();
      display.drawString(0, 0, "t_power");
      display.drawString(35, 0, " ");
      display.drawString(70, 0, "S1");
      display.drawString(100, 0, "S2");
      // output
      display.drawString(0, 20, String(t_power));
      display.drawString(35, 20, " ");
      display.drawString(70, 20, String(t_Serv_L));
      display.drawString(100, 20, String(t_Serv_R));

    } else if (screen_page == 9) {
      //    display.clear();
      display.setFont(ArialMT_Plain_16);
      display.clear();
      display.drawString(0, 0, "tilt X");
      display.drawString(35, 0, " ");
      display.drawString(70, 0, "tilt Y");
      display.drawString(100, 0, "S2");
      // output
      display.drawString(0, 20, String(t_tilt_X));
      display.drawString(35, 20, " ");
      display.drawString(70, 20, "" );
      display.drawString(100, 20, String(t_tilt_Y));

    }

    else if (screen_page == 255) {


      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "Motors Calibration!");
      display.setFont(ArialMT_Plain_24);
      display.drawString(40, 20, String(map(Calib, 0, 255, -128, 128)));

    } else {
      display.clear();
      DispMainScreen();
      DispMainScreenData();



    }
    if (ScreenSaverTimer < SCREENSAVER_TIME) {
      display.display();
    } else
      display.clear();
    display.display();
  };


  // MAIN CODE
  if (currentMillis - previousMillis >= INTERVAL) {
    if (!digitalRead(STARTER_PIN)) {
      Serial.println("BUUTON PRESSED");
    };
    TelemetryCounter++;
    display.clear();
    previousMillis = currentMillis;

    // przesyl danych
    if (digitalRead(EN_PIN) && !digitalRead(STARTER_PIN)) {
      starter_en = true;
    };
    if (!starter_en) {
      if (digitalRead(EN_PIN) or Joy2.getX() > 10 or Joy2.getX() < -10 or Joy2.getY() > 10 or Joy2.getY() < -10) {

        if (digitalRead(EN_PIN)) {
          mot_power = bitWrite(mot_power, 0, 1);
        };
        SendData(false);
        DataCounter = 0;
        // en pin ON instructions
        if ((Joy1.getB() and !Joy2.getB()) or (!Joy1.getB() and Joy2.getB())) {

          screen_page = 255;
          Serial.print("Recalibrating:");
          Serial.println(Calib);
        }

        if (Joy1.getB()) {
          if (Calib > 0) {
            Calib--;
          }
        };
        if (Joy2.getB()) {
          if (Calib < 255) {
            Calib++;
          }
        };
      }
      else  {

        mot_power = 0;

        if (DataCounter < 3)
        {
          SendData(false);
          DataCounter++;
        };
        // EN PIN OFF INSTRUCTIONS
        if (Joy2.getB()) {
          screen_page++;
          Serial.print("Screen page:");
          Serial.println(screen_page);
          if (screen_page > 9) {
            screen_page = 0;
          };
          delay(500);
        };

        if (Joy1.getB()) {
          if (screen_page == 1) {
            ASS_en = !ASS_en;
            bitWrite(mot_power, 1, ASS_en);
          };

          if (screen_page == 2) {
            AFS_en = !AFS_en;
            bitWrite(mot_power, 2, AFS_en);
          };

          if (screen_page == 3) {
            AP_en = !AP_en;
            bitWrite(mot_power, 3, AP_en);
          };

          if (screen_page == 4) {
            TH_en = !TH_en;
            bitWrite(mot_power, 4, TH_en);
          };
          delay(500);
        };

      };
      starter_timer = 0;
    }
    else {
      SendData(false);
      starter_timer++;

      if (starter_timer > STARTER_MULTIPLIER) {
        starter_en = false;
        starter_timer = 0;
        Serial.println("Starting procedure... FINISHED");
      };
      if (!digitalRead(STARTER_PIN) && starter_timer > 10) {
        starter_en = false;
        starter_timer = 0;
        Serial.println("Starting procedure... BREAK!");
      };

    }

    // debugging

    //    if (digitalRead(EN_PIN)) {
    //      Serial.println("");
    //      //      for (int i = 0; i <= 9; i++) {
    //      //        Serial.print(msg[i]);
    //      //        Serial.print(",");
    //      //      };
    //      for (int i = 0; i <= 7; i++)
    //      {
    //        Serial.print(i);
    //        Serial.print("-");
    //        Serial.print(bitRead(mot_power, i));
    //        Serial.print(", ");
    //
    //      };
    //      Serial.println();
    //    };



    //  delay(500);

  }

  //    Serial.print("Buttons:");
  //    if (Joy1.getB()) {
  //      Serial.print("T");
  //    }
  //    else {
  //      Serial.print("F");
  //    }
  //
  //    Serial.print(",");
  //    if (Joy2.getB()) {
  //      Serial.print("T");
  //    }
  //    else {
  //      Serial.print("F");
  //    }
  // TELEMETRY REQUEST
  if (TelemetryCounter > TelemetryMultiplier) {
    TelemetryCounter = 0;
    TelemetryErrorCounter ++;
    if (TelemetryErrorCounter > 16 * TelemetryThreshold) {
      TelemetryErrorCounter = 0;
      t_power = 0;
      t_mot_L = 0;
      t_mot_R = 0;
      t_b_lev = 0 ;
      t_Serv_L = 0;
      t_Serv_R = 0;
    };
    SendData(true);

    //      if (TelemetryErrorCounter < 4 * TelemetryThreshold) {
    //        if (digitalRead(EN_PIN)) {
    //          mot_power = 2;
    //          //
    //        } else {
    //          mot_power = 12;
    //        };
    //        SendData();
    //        TelemetryErrorCounter = 0;
    //      } else if (TelemetryErrorCounter > 4 * TelemetryThreshold and digitalRead(EN_PIN))
    //      {
    //        TelemetryErrorCounter = 0;
    //      };
    //    };
  }







}






