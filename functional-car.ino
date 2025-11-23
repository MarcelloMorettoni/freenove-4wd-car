/**********************************************************************
  Filename      : IR_Car_Smart_Avoidance.ino
  Modification  : Look Left/Right and Decide
**********************************************************************/
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <IRremote.hpp>
#include "Freenove_4WD_Car_For_Pico_W.h"
#include "Freenove_4WD_Car_Emotion.h"
#include "Freenove_VK16K33_Lib.h"
#include "Freenove_4WD_Car_WS2812.h"

#define IR_Pin 3  
#define ENABLE_LED_FEEDBACK          true
#define DISABLE_LED_FEEDBACK         false

// ***** SETTINGS *****
#define SERVO_CENTER_ANGLE  135  // Your calibrated center
#define STOP_DISTANCE       30   // Distance to stop (cm). Change to 300 if using mm.
#define TURN_SPEED          50   // How fast to turn
#define DRIVE_SPEED         40   // How fast to drive forward
// ********************

// --- VARIABLES ---
static int servo_1_angle = SERVO_CENTER_ANGLE;
int emotion_flag = 0;
int ws2812_flag = 0;
int CAR_MODE_VOL = 0;
int LASt_CAR_MODE_VOL = 0;

unsigned long last_received_time = 0; 
unsigned long lastSonarTime = 0;   
bool isAutoMode = false;           
float currentDistance = 0; 

void setup() {
  Serial.begin(115200);  
  Motor_Setup();         
  Servo_Setup();         
  Buzzer_Setup();        
  WS2812_Setup();        
  Emotion_and_Ultrasonic_Setup();
  IrReceiver.begin(IR_Pin, DISABLE_LED_FEEDBACK); 
  
  Servo_1_Angle(servo_1_angle);
  delay(100);
}

void loop() {
  
  // 1. ALWAYS READ SENSOR (Every 50ms)
  if (millis() - lastSonarTime > 50) {
      lastSonarTime = millis();
      currentDistance = Get_Sonar();
      // Debug print
      Serial.print("Dist: ");
      Serial.println(currentDistance);
  }

  // 2. Check for IR Commands
  if (IrReceiver.decode()) {
      unsigned long value = IrReceiver.decodedIRData.decodedRawData;
      if (value != 0xFFFFFFFF) {
          handleControl(value);
      }
      last_received_time = millis(); 
      IrReceiver.resume(); 
  }

  // 3. Manual Mode Safety Stop
  if (!isAutoMode && (millis() - last_received_time > 150)) {
      Motor_Move(0, 0);
  }

  // 4. Auto Mode Logic
  if (isAutoMode) {
      runAutoMode();
  }

  // 5. Update displays
  Emotion_and_Ultrasonic_Setup();
  oa_CalculateVoltageCompensation();
  if (Check_Module_value == MATRIX_IS_EXIST) {
    Emotion_Show(emotion_task_mode); 
  } 
  Car_Select(carFlag);
  WS2812_Show(ws2812_task_mode);    
}

// --- NEW SMART AUTO MODE ---
void runAutoMode() {
    
    // Condition 1: OBSTACLE DETECTED
    // If distance is valid (>0) AND closer than limit
    if (currentDistance > 0 && currentDistance < STOP_DISTANCE) {
        
        Serial.println("Obstacle! Starting Avoidance Routine...");
        performSmartAvoidance(); // Enter the "Thinking" function
    } 
    // Condition 2: CLEAR PATH
    else {
        Motor_Move(DRIVE_SPEED, DRIVE_SPEED); 
    }
}

// --- THE "THINKING" FUNCTION ---
void performSmartAvoidance() {
    // 1. STOP
    Motor_Move(0, 0);
    Buzzer_Variable(2000, 50, 1); // Short beep
    delay(200);

    // 2. LOOK LEFT
    Servo_1_Angle(170); // Look all the way left
    delay(400);         // Wait for servo to move
    float leftDist = Get_Sonar();
    Serial.print("Left Dist: "); Serial.println(leftDist);

    // 3. LOOK RIGHT
    Servo_1_Angle(10);  // Look all the way right
    delay(400);         // Wait for servo to move
    float rightDist = Get_Sonar();
    Serial.print("Right Dist: "); Serial.println(rightDist);

    // 4. RESET EYES
    Servo_1_Angle(SERVO_CENTER_ANGLE);
    delay(300);

    // 5. DECIDE
    
    // If BOTH are blocked (less than limit), we must go BACK
    if ((leftDist < STOP_DISTANCE || leftDist == 0) && (rightDist < STOP_DISTANCE || rightDist == 0)) {
       Serial.println("Trapped! Backing up.");
       
       // Back up
       Motor_Move(-DRIVE_SPEED, -DRIVE_SPEED);
       delay(600); 
       
       // Turn around (180 spin)
       Motor_Move(-TURN_SPEED, TURN_SPEED); 
       delay(500);
    }
    // If Left is better than Right
    else if (leftDist > rightDist) {
       Serial.println("Turning Left.");
       Motor_Move(-TURN_SPEED, TURN_SPEED); // Turn Left
       delay(400); // Adjust this delay to change how much it turns (90 degrees?)
    }
    // If Right is better than Left
    else {
       Serial.println("Turning Right.");
       Motor_Move(TURN_SPEED, -TURN_SPEED); // Turn Right
       delay(400); 
    }
    
    // Stop briefly before resuming loop to stabilize
    Motor_Move(0, 0);
    delay(100);
}

void handleControl(unsigned long value) {
  switch (value) {
    // --- MOVEMENT COMMANDS (Disable Auto Mode) ---
    case 0xBF40FF00: isAutoMode = false; Motor_Move(50, 50); break;   // +
    case 0xE619FF00: isAutoMode = false; Motor_Move(-50, -50); break; // -
    case 0xF807FF00: isAutoMode = false; Motor_Move(-50, 50); break;  // Left
    case 0xF609FF00: isAutoMode = false; Motor_Move(50, -50); break;  // Right
    case 0xEA15FF00: isAutoMode = false; Motor_Move(0, 0); break;     // Stop
      
    // --- TEST BUTTON: TOGGLE AUTO MODE ---
    case 0xBB44FF00:  // 'TEST'
      if (isAutoMode) {
        isAutoMode = false;
        Motor_Move(0, 0);           
        Buzzer_Variable(2000, 100, 2); 
      } 
      else {
        isAutoMode = true;
        Servo_1_Angle(SERVO_CENTER_ANGLE);     
        Buzzer_Variable(2000, 100, 1); 
      }
      break;
      
    // --- OTHER COMMANDS ---
    case 0xE916FF00: servo_1_angle += 10; Servo_1_Angle(servo_1_angle); break; // 0
    case 0xF30CFF00: servo_1_angle -= 10; Servo_1_Angle(servo_1_angle); break; // 1
    case 0xF708FF00: servo_1_angle = SERVO_CENTER_ANGLE; Servo_1_Angle(servo_1_angle); break; // 4
    case 0xF20DFF00: isLightModeFirstStarting = true; Servo_1_Angle(SERVO_CENTER_ANGLE); Car_SetMode(1); break; // C
    case 0xA15EFF00: Servo_1_Angle(SERVO_CENTER_ANGLE); Car_SetMode(2); break; // 3
    case 0xA55AFF00: // 6
      Servo_1_Angle(SERVO_CENTER_ANGLE);
      if (Check_Module_value == SONAR_IS_ESIST) { LASt_CAR_MODE_VOL = 1; Car_SetMode(3); }
      else { Buzzer_Variable(2000, 50, 2); Car_SetMode(0); Motor_Move(0, 0); }
      break;
    case 0xB54AFF00: emotion_flag = 0; Emotion_SetMode(emotion_flag); Servo_1_Angle(SERVO_CENTER_ANGLE); Car_SetMode(0); Motor_Move(0, 0); break; // 9
    case 0xE718FF00: // 2
      if (Check_Module_value == MATRIX_IS_EXIST) { emotion_flag++; if (emotion_flag > 7) emotion_flag = 0; Emotion_SetMode(emotion_flag); } 
      else { Buzzer_Variable(2000, 50, 2); }
      break;
    case 0xE31CFF00: emotion_flag = 0; Emotion_SetMode(emotion_flag); break; // 5
    case 0xBD42FF00: ws2812_flag++; if (ws2812_flag >= 6) ws2812_flag = 0; WS2812_SetMode(ws2812_flag); break; // 7
    case 0xAD52FF00: ws2812_flag = 0; WS2812_SetMode(ws2812_flag); break; // 8
    case 0xFFFFFFFF: break;
    default: break;
  }
}
