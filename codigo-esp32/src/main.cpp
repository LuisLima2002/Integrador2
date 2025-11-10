#include <main.h>

// ========= HCSR04 =========
// enable distance sensor
HCSR04 hc(TRIG_PIN, ECHO_PIN); // (trig, echo)
// BluetoothSerial SerialBT;
void setup()
{
  Serial.begin(9600);
  // SerialBT.begin("ESP32_BT_Device");
  // enable PWM AND motors pins
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  ledcSetup(CH_M1, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_EN, CH_M1);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RES);
  ledcAttachPin(M2_EN, CH_M2);

  // enable Enconders pind and set interruptions
  // encoder_begin();

  // enable softAP and server
  // web_begin();

  //enable IMU
  MPU_begin();

  // set state to configure
  gps_begin(&Serial2, GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);
  gps_set_target(TARGET_LAT, TARGET_LON, TARGET_INNER_M, TARGET_OUTER_M);

  Serial.println("Setup done");
  state = MachineState::Forward;
  Serial.println("State: Forward");
  digitalWrite(COLLISION_LED, HIGH);
  delay(10000);
  digitalWrite(COLLISION_LED, LOW);
  reset_yaw();
}

// state:

// Config
// Wait connections to GPS, when finished go to DirectionCalc

// ReferenceFetch
// Save current gps position as p0 and turn motor to go forward, after X seconds stop motos and go to RouteCalc

// RouteCalc
// save current gps position as p1 and using p0, p1 and p_ref set the theta_togo and distance_togo, also set p1=p0. After this go to Rotate

// Rotate
// Rotate theta_togo angles in the right direction, after this go to forward

// Forward
//  Move distance_togo forwad, after this go to Evaluation

// Evaluation
// Check if current position is close enough to p_ref go to done, else save currente position as p0 and go to route calc

// Done
// Buzzer on and do not move anymore

unsigned long lastTime = 0;
float yaw = 0;
float kmh = 0;
GpsNavCommand navCmd;
unsigned long previousMillisRPM;
void loop()
{
  // if (SerialBT.available()) {
  //   // Read the incoming byte and print it to the USB Serial Monitor
  //   char incomingChar = SerialBT.read();
  //   Serial.print("Received from Python: ");
  //   Serial.write(incomingChar);
  // }

  yaw = update_yaw();
  // unsigned long now = millis();
  // if (now - previousMillisRPM >= intervalMs) {
  //       previousMillisRPM = now;
  //   kmh = update_speed();
  // }
  float dist_cm = hc.dist();
  if (dist_cm > 2.0f && dist_cm < 25.0f)
  {
    Serial.println("BLOCKED");
    stopMotors();

    digitalWrite(COLLISION_BUZZER, HIGH);
    digitalWrite(COLLISION_LED, HIGH);
    delay(2000);
    return;
  }

  // Finite machine state
  switch (state)
  {
  case MachineState::SaveP0:
    gps_feed();
    if(gps_save_p0()){
      state = MachineState::Forward3Seconds;
      Serial.println("State: Forward3Seconds");
      lastTime = millis();
      reset_yaw();
    }
    break;

  case MachineState::Forward3Seconds:
    driveStraight(yaw, 180);
  //   if ()
    //   {
    //     state = MachineState::SaveP1;
    //     Serial.println("State: SaveP1");
    // while(true){}

    //   }

    break;

  case MachineState::SaveP1:
  gps_feed();
  if(gps_save_p1()){
      state = MachineState::RouteCalc;
      Serial.println("State: RouteCalc");

    }
  break;

  case MachineState::RouteCalc:
    navCmd = gps_calculate_route();
    Serial.print("distance:");
    Serial.print(navCmd.distance_m,5);
    Serial.print(" ANGLE:");
    Serial.println(navCmd.bearing_to_goal_deg,5);
    reset_yaw();
    resetTimer();
    state = MachineState::Rotate;
    break;

  case MachineState::Rotate:
    if(turnByAngle(180,yaw)){
      state = MachineState::Forward;

      reset_yaw();
      // resetDistance();
      // resetTimer();
    }
    break;

  case MachineState::Forward:
      driveStraight(yaw, 180);
      // setMotor(1,255);
      // setMotor(2,-255);
    break;

  case MachineState::Evaluation:
    if(gps_evaluate()){
      state = MachineState::Done;
    }else{
      digitalWrite(COLLISION_BUZZER, HIGH);
      digitalWrite(COLLISION_LED, HIGH);
    };
    break;

  // case MachineState::Done:
  //   /* code */
  //   break;
  }

  delay(50);
}
