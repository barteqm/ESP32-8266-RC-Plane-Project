#define CALIB_COUNT 20
#define TOLERANCE 10
class MyJoystick {

    int pin_X;
    int pin_Y;
    int pin_B;
    int cal_0_X = 0;
    int cal_0_Y = 0;
    int max_X = 4095;
    int max_Y = 4095;
    int dev_X = 0;
    int dev_Y = 0;
    int maximum_X = 0;
    int maximum_Y = 0;
    int minimum_X = 0;
    int minimum_Y = 0;

  public:
    MyJoystick(int pinX, int pinY, int pinB) {
      pin_X = pinX;
      pin_Y = pinY;
      pin_B = pinB;

      pinMode(pin_X, INPUT);
      pinMode(pin_Y, INPUT);
      pinMode(pin_B, INPUT_PULLUP);

    };
    void calibrate (int maxX, int maxY) {
      max_X = maxX;
      max_Y = maxY;

      maximum_X = 0;
      maximum_Y = 0;
      minimum_X = 4096;
      minimum_Y = 4096;
      // gathering calib values
      Serial.print("Calibrating Joystick(do not touch controls)");
      for (int i = 0; i < CALIB_COUNT; i++) {
        int tmp_X = analogRead(pin_X);
        int tmp_Y = analogRead(pin_Y);
        //average
        cal_0_X = cal_0_X + tmp_X;
        cal_0_Y = cal_0_Y + tmp_Y;

        // min/max

        if (tmp_X > maximum_X) {
          maximum_X = tmp_X;
        };

        if (tmp_X < minimum_X) {
          minimum_X = tmp_X;
        };
        if (tmp_Y > maximum_Y) {
          maximum_Y = tmp_Y;
        };
        if (tmp_Y < minimum_Y) {
          minimum_Y = tmp_Y;
        };
        delay(500);
        Serial.print(".");
      };
      // calculating averages
      cal_0_X = cal_0_X / CALIB_COUNT;
      cal_0_Y = cal_0_Y / CALIB_COUNT;
      dev_X = maximum_X - minimum_X;
      dev_Y = maximum_Y - minimum_Y;

      // report:
      Serial.println("");
      Serial.println("Calibration Finished");
      Serial.print("Avg X: ");
      Serial.print(cal_0_X);
      Serial.print(" Avg Y: ");
      Serial.print(cal_0_Y);
      Serial.print(" Max X: ");
      Serial.print(maximum_X);
      Serial.print(" Max Y: ");
      Serial.println(maximum_Y);
      Serial.print(" Min X: ");
      Serial.print(minimum_X);
      Serial.print(" Min Y: ");
      Serial.println(minimum_Y);
      Serial.print(" Dev X: ");
      Serial.print(dev_X);
      Serial.print(" Dev Y: ");
      Serial.println(dev_Y);
   //   delay(5000);
    };
    int getX () {
      int X = analogRead(pin_X);
      if (true) { //X > maximum_X+ TOLERANCE or X < minimum_X-TOLERANCE
        X = X - cal_0_X;
        X = map(X, cal_0_X*-1, cal_0_X, max_X * -1, max_X);
        if (X > max_X) {
          X = max_X;
        };
        if (X < -1 * max_X) {
          X = -max_X;
        };
        return X;
      } else return 0;
      //  return X_out;
    };
    int getY () {
      int Y = analogRead(pin_Y);
      if (true) {//Y > maximum_Y+TOLERANCE or Y < minimum_Y-TOLERANCE
        //        Serial.print("Y VAL:");
        //        Serial.println(Y);
        //        Y = map(Y  - cal_0_Y, cal_0_Y * -1, cal_0_Y, cal_0_Y * -1, cal_0_Y);
        //        Serial.print("Y VAL After1:");
        //        Serial.println(Y);
        //        Y = map(Y, cal_0_Y * -1, cal_0_Y, max_Y * -1, max_Y);
        //        Serial.print("Y VAL After2:");
        //        Serial.println(Y);
        Y = Y - cal_0_Y;
        Y = map(Y, cal_0_Y*-1, cal_0_Y, max_Y * -1, max_Y);
        if (Y > max_Y) {
          Y = max_Y;
       //   Serial.print("OVERLOAD H");
        };
        if (Y < -1 * max_Y) {
          Y = -max_Y;
      //    Serial.print("OVERLOAD L");
        };
        return Y;
      } else return 0;
    };
    bool getB() {

      return not digitalRead(pin_B);
    }
};

