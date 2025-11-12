#include <main.h>
#include "BluetoothSerial.h"

// ========= HCSR04 =========
// enable distance sensor
HCSR04 hc(TRIG_PIN, ECHO_PIN); // (trig, echo)
BluetoothSerial SerialBT;
void setup()
{
  Serial.begin(9600);
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


  SerialBT.begin("ESP32_Device");

  while (true) {
    gps_feed();
    // Check if there is any data from the phone
    if (SerialBT.available()) {
      // Read the incoming message as a String
      String incomingMessage = SerialBT.readString();
      
      // Remove any leading/trailing whitespace (like newlines)
      incomingMessage.trim();
      
      // Print the received message to the Serial Monitor for debugging
      Serial.print("Received: ");
      Serial.println(incomingMessage);

      // Check if the message is "start" (case-insensitive)
      if (incomingMessage.equalsIgnoreCase("start")) {
        break;
      }
    }
    
    // A small delay to prevent the ESP32 from crashing
    delay(100); 
  }

  SerialBT.println("Setup done");
  state = MachineState::SaveP0;
  SerialBT.println("State: SaveP0");
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
  //Positivo vira para direita
  yaw = update_yaw();
  gps_feed();
  // if (gps_evaluate()) Serial.println("Dentro!");
  // else Serial.println("Fora!");
  // return;
  // unsigned long now = millis();
  // if (now - previousMillisRPM >= intervalMs) {
  //       previousMillisRPM = now;
  //   kmh = update_speed();
  // }
  // float dist_cm = hc.dist();
  // if (dist_cm > 2.0f && dist_cm < 25.0f)
  // {
  //   Serial.println("BLOCKED");
  //   stopMotors();
  //   Serial.println(dist_cm);
  //   digitalWrite(COLLISION_BUZZER, HIGH);
  //   digitalWrite(COLLISION_LED, HIGH);
  //   delay(1000);
  //   return;
  // }

  // Finite machine state
  switch (state)
  {
  case MachineState::SaveP0:
    if(gps_save_p0()){
      SerialBT.print("P0 saved at Lat: ");
      SerialBT.print(get_p0_lat(),6);
      SerialBT.print(" Lon: ");
      SerialBT.print(get_p0_lon(),6);
      SerialBT.print(" Satellites: ");
      SerialBT.print(gps_satellites());
      SerialBT.print(" HDOP: ");
      SerialBT.println(gps_hdop());
      
      state = MachineState::Forward3Seconds;
      SerialBT.println("State: Forward");
      lastTime = millis();
      reset_yaw();
    }else{
      SerialBT.println("Waiting for a good gps read");
      SerialBT.print(" Satellites: ");
      SerialBT.print(gps_satellites());
      SerialBT.print(" HDOP: ");
      SerialBT.println(gps_hdop());
      delay(1000);
    }
    break;

  case MachineState::Forward3Seconds:
    driveStraight(yaw, 0);
    if (lastTime + 20000 < millis())
      {
        stopMotors();
        state = MachineState::SaveP1;
        SerialBT.println("State: SaveP1");
        gps_loop_3_second();
      }

    break;

  case MachineState::SaveP1:
  if(gps_save_p1()){
      SerialBT.print("P1 saved at Lat: ");
      SerialBT.print(get_p1_lat(),6);
      SerialBT.print(" Lon: ");
      SerialBT.println(get_p1_lon(),6);
      SerialBT.print(" Satellites: ");
      SerialBT.print(gps_satellites());
      SerialBT.print(" HDOP: ");
      SerialBT.println(gps_hdop());

      state = MachineState::RouteCalc;
      Serial.println("State: RouteCalc");
    }else{
      SerialBT.println("Waiting for a good gps read");
      SerialBT.print(" Satellites: ");
      SerialBT.print(gps_satellites());
      SerialBT.print(" HDOP: ");
      SerialBT.println(gps_hdop());
      delay(1000);
    }
  break;

  case MachineState::RouteCalc:
    navCmd = gps_calculate_route();
    SerialBT.print("Bearing: ");
    SerialBT.print(navCmd.bearing_to_goal_deg);
    SerialBT.print(" Distance: ");
    SerialBT.println(navCmd.distance_m);
    reset_yaw();
    lastTime = millis();
    state = MachineState::DriveToGoal;
    Serial.println("State: DriveToGoal");
    break;
case MachineState::DriveToGoal:
    // Check if 10 seconds (10000 milliseconds) have passed
    if (millis() - lastTime > 10000) 
    {
        // 10 seconds are up. Stop the robot.
        stopMotors();
        
        // Go directly to Evaluation state
        state = MachineState::Evaluation;
        SerialBT.println("State: Evaluation");
        reset_yaw(); // Reset yaw for the next potential action
    } 
    else 
    {
        // 10 seconds have not passed yet, keep driving
        driveStraight(yaw, navCmd.bearing_to_goal_deg);
    }
    break;

  case MachineState::Evaluation:
    if(gps_evaluate()){
      state = MachineState::Done;
      SerialBT.println("State: Done");
    }else{
      SerialBT.println("Saving new P0 as P1");
      p1_to_p0();
      gps_loop_3_second();
      state = MachineState::SaveP1;
      SerialBT.println("State: SaveP1");
    };
    break;

  case MachineState::Done:
    // digitalWrite(COLLISION_BUZZER, HIGH);
    setMotor(1,175);
    setMotor(2,-175);
    break;
  }

  delay(50);
}
