#define MOTOR_ENABLE_PIN D4
#define MOT_L_PIN D7
#define MOT_R_PIN D8

void Motors(bool mot_power_IN, int X_IN, int Y_IN, int T_IN) {

  if (mot_power_IN) {


    int x_diff = map(X_IN, -255, 255, T_IN * -1, T_IN);
    int y_diff = map(Y_IN, -255, 255, T_IN * -1, T_IN);
    //    mot_L = T - x_diff + y_diff - CALIB/2;
    //    mot_R = T + x_diff + y_diff + CALIB/2;

    if (X_IN > 0) {
      mot_L = T_IN - CALIB_L + y_diff ;
      mot_R = T_IN - x_diff + y_diff - CALIB_R;
    } else if (X_IN < 0) {
      mot_L = T_IN + x_diff + y_diff - CALIB_L;
      mot_R = T_IN - CALIB_R + y_diff ;
    } else {
      mot_L = T_IN - CALIB_L + y_diff ;
      mot_R = T_IN - CALIB_R + y_diff;
    };


    if (mot_L > 0 or mot_R > 0) {
      //   digitalWrite(MOTOR_ENABLE_PIN, 1);
      motors = true;
    }
    else {
      // digitalWrite(MOTOR_ENABLE_PIN, 0);
      motors = false;
    };

    // motor max/min conditions
    if (mot_L >= 1023) {
      digitalWrite(MOT_L_PIN, HIGH);
      mot_L = 1023;
    };
    if (mot_L <= 0) {
      digitalWrite(MOT_L_PIN, LOW);
      mot_L = 0;
    };
    if (mot_R >= 1023) {
      digitalWrite(MOT_R_PIN, HIGH);
      mot_R = 1023;
    };
    if (mot_R <= 0) {
      digitalWrite(MOT_R_PIN, LOW);
      mot_R = 0;
    };

    // motors normal conditions
    if (mot_L > 0 and mot_L < 1023)
    {
      // mot_l=gamma10[mot_L];
      analogWrite(MOT_L_PIN, mot_L);
    };
    if (mot_R > 0 and mot_R < 1023) {
       // mot_R=gamma10[mot_R];
      analogWrite(MOT_R_PIN, mot_R);
    };

  }
  else {

    analogWrite(MOT_L_PIN, 0);
    analogWrite(MOT_R_PIN, 0);
    //   digitalWrite(MOTOR_ENABLE_PIN, 0);

    mot_L = 0;
    mot_R = 0;
  }
}


void Servoes (uint8_t X_IN, uint8_t Y_IN) {
  Serv_DIFF = map(Y_IN, 0, 255, -127, 127);
  ServL_VAL = map(X_IN + Serv_DIFF, 0, 255, Serv_1_MIN, Serv_1_MAX);
  ServR_VAL = map((X_IN - Serv_DIFF) * -1, -255, 0, Serv_1_MIN, Serv_1_MAX) ;


  if (ServR_VAL > Serv_1_MAX) {
    ServR_VAL = Serv_1_MAX;
  };
  if (ServR_VAL < Serv_1_MIN) {
    ServR_VAL = Serv_1_MIN;
  };


  if (ServL_VAL > Serv_1_MAX) {
    ServL_VAL = Serv_1_MAX;
  };
  if (ServL_VAL < Serv_1_MIN) {
    ServL_VAL = Serv_1_MIN;
  };

  ServL.write(ServL_VAL + ServL_CAL);
  ServR.write(ServR_VAL + ServR_CAL);
}

class Flasher
{
    // Class Member Variables
    // These are initialized at startup
    int ledPin;      // the number of the LED pin
    long OnTime;     // milliseconds of on-time
    long OffTime;    // milliseconds of off-time

    // These maintain the current state
    int ledState;                 // ledState used to set the LED
    unsigned long previousMillis;   // will store last time LED was updated

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public:
    Flasher(int pin, long on, long off)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);

      OnTime = on;
      OffTime = off;

      ledState = LOW;
      previousMillis = 0;
    }

    void Update()
    {
      // check to see if it's time to change the state of the LED
      unsigned long currentMillis = millis();

      if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = LOW;  // Turn it off
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(ledPin, ledState);  // Update the actual LED
      }
      else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
      {
        ledState = HIGH;  // turn it on
        previousMillis = currentMillis;   // Remember the time
        digitalWrite(ledPin, ledState);   // Update the actual LED
        //     Serial.println("FLASH ALERT");
        // ALERT = true;
      }
    }
    void Off()
    {
      if (ledState == HIGH) {
        digitalWrite(ledPin, LOW);
        //    ALERT = false;
      };

    }
};
