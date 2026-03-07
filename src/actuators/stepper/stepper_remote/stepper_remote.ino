// --- Pin Definitions ---
#define EN1   7  // Enable Motor 1
#define DIR1  8  // Direction Motor 1
#define ST1   9  // Step Motor 1

#define EN2   5  // Enable Motor 2
#define DIR2  4  // Direction Motor 2
#define ST2   3  // Step Motor 2

#define ADC_PIN A0 // ADC input pin for voltage monitoring

// --- Motor & Homing Configuration ---
#define MICROSTEP_FACTOR       16
#define FULL_TRAVEL_STEPS      1000 // The physical limit (0 to 1000)
#define FULL_TRAVEL_MICROSTEPS (FULL_TRAVEL_STEPS * MICROSTEP_FACTOR)
#define HOMING_FREQUENCY_HZ    8000 // Speed in microsteps/sec

// --- Safety Configuration ---
#define VOLTAGE_THRESHOLD 5.0 // Minimum voltage required to enable motors

// --- Global Position & Target Tracking ---
long motor1_current_pos = 0;
long motor2_current_pos = 0;
long motor1_target_pos = 0;
long motor2_target_pos = 0;

// --- Non-Blocking Timing Variables ---
unsigned long motor_step_delay_us = 0; // 0 means motors are idle
unsigned long m1_last_step_time = 0;
unsigned long m2_last_step_time = 0;
bool move_complete_msg_sent = true; // Flag to prevent spamming "Move complete"

// --- Setup ---
void setup() {
  Serial.begin(115200);

  // Configure all pins
  pinMode(EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(ST1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(ST2, OUTPUT);
  pinMode(ADC_PIN, INPUT);

  digitalWrite(EN1, LOW); // Start with motors disabled
  digitalWrite(EN2, LOW);

  Serial.println("--- Arduino Ready. Send <HOME> to begin homing sequence. ---");
  waitForHomeCommand();

  // Safety Check: Wait here until the main 12V power is connected.
  while (!isMainPowerOn()) {
    Serial.println("ERR: Main 12V power is off. Please connect power to continue.");
    delay(2000); // Wait before checking again to avoid spamming serial
  }

  // --- Homing Sequence (Blocking is acceptable here) ---
  performHoming();

  Serial.println("--- Homing Complete. Ready for motion commands. ---");
}

// --- Main Loop ---
void loop() {
  handleSerialCommands();
  updateMotors();
}

// --- Task Functions ---

/**
 * @brief Checks for and parses new serial commands, including safety checks.
 */
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Check for STOP command (stops motion, disables motors)
    if (command == "<STOP>") {
      motor1_target_pos = motor1_current_pos;
      motor2_target_pos = motor2_current_pos;
      motor_step_delay_us = 0;
      digitalWrite(EN1, LOW);
      digitalWrite(EN2, LOW);
      Serial.println("STOP received. Motion halted, motors disabled.");
      move_complete_msg_sent = true;
      return;
    }

    if (command.startsWith("<") && command.endsWith(">")) {
      // 1. Power Safety Check
      if (!isMainPowerOn()) {
        Serial.println("ERR: Main power lost. Move cancelled.");
        return; 
      }
      
      String content = command.substring(1, command.length() - 1);
      long target_m1, target_m2, temp_freq;
      int microsteps;

      int parsed_items = sscanf(content.c_str(), "%ld,%ld,%ld,%d", &target_m1, &target_m2, &temp_freq, &microsteps);

      if (parsed_items == 4) {
        // 2. Positional Limit Safety Check
        if (target_m1 < 0 || target_m1 > FULL_TRAVEL_STEPS ||
            target_m2 < 0 || target_m2 > FULL_TRAVEL_STEPS) {
          Serial.println("ERR: Target out of physical limits (0-1000).");
          return;
        }

        digitalWrite(EN1, HIGH); 
        digitalWrite(EN2, HIGH);
        
        motor1_target_pos = target_m1 * microsteps;
        motor2_target_pos = target_m2 * microsteps;

        float frequency_in_microsteps = (float)temp_freq * microsteps;
        motor_step_delay_us = (frequency_in_microsteps > 0) ? (1000000 / frequency_in_microsteps) : 0;

        digitalWrite(DIR1, (motor1_target_pos > motor1_current_pos) ? HIGH : LOW);
        digitalWrite(DIR2, (motor2_target_pos > motor2_current_pos) ? HIGH : LOW);

        move_complete_msg_sent = false; 
      } else {
        Serial.println("ERR: Invalid command format.");
      }
    }
  }
}

/**
 * @brief Steps the motors if they need to move. This function is called on every
 * loop cycle to ensure continuous, non-blocking movement.
 */
void updateMotors() {
  if (motor_step_delay_us == 0) return;

  if (motor1_current_pos == motor1_target_pos && motor2_current_pos == motor2_target_pos) {
    if (!move_complete_msg_sent) {
      digitalWrite(EN1, LOW);
      digitalWrite(EN2, LOW);
      motor_step_delay_us = 0;
      Serial.println("Move complete.");
      move_complete_msg_sent = true;
    }
    return;
  }

  unsigned long now = micros();

  if (motor1_current_pos != motor1_target_pos && (now - m1_last_step_time >= motor_step_delay_us)) {
    m1_last_step_time = now;
    digitalWrite(ST1, HIGH);
    delayMicroseconds(5);
    digitalWrite(ST1, LOW);
    motor1_current_pos += (motor1_target_pos > motor1_current_pos) ? 1 : -1;
  }

  if (motor2_current_pos != motor2_target_pos && (now - m2_last_step_time >= motor_step_delay_us)) {
    m2_last_step_time = now;
    digitalWrite(ST2, HIGH);
    delayMicroseconds(5);
    digitalWrite(ST2, LOW);
    motor2_current_pos += (motor2_target_pos > motor2_current_pos) ? 1 : -1;
  }
}

// --- Helper Functions ---

/**
 * @brief Checks if the main power supply is active by reading the ADC voltage.
 * @return True if voltage is above the threshold, false otherwise.
 */
bool isMainPowerOn() {
  int sensorValue = analogRead(ADC_PIN);
  float voltage = sensorValue * 5.0 * 11 / 1023.0;
  return voltage > VOLTAGE_THRESHOLD;
}

/**
 * @brief Waits for the initial <HOME> command from the Python script.
 */
void waitForHomeCommand() {
  String command = "";
  while (command != "<HOME>") {
    if (Serial.available() > 0) {
      command = Serial.readStringUntil('\n');
      command.trim();
    }
  }
}

/**
 * @brief Runs the initial homing sequence.
 */
void performHoming() {
  Serial.println("--- Homing sequence started... ---");
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(10);
  
  blockingMoveTo(FULL_TRAVEL_MICROSTEPS, FULL_TRAVEL_MICROSTEPS, HOMING_FREQUENCY_HZ);
  delay(200);
  blockingMoveTo(0, 0, HOMING_FREQUENCY_HZ);
  
  motor1_current_pos = 0;
  motor2_current_pos = 0;
  motor1_target_pos = 0;
  motor2_target_pos = 0;
  
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
}

/**
 * @brief A blocking move function used ONLY for the initial homing sequence.
 */
void blockingMoveTo(long m1_target, long m2_target, float frequency_hz) {
  unsigned long step_delay = 1000000 / frequency_hz;
  digitalWrite(DIR1, (m1_target > motor1_current_pos) ? HIGH : LOW);
  digitalWrite(DIR2, (m2_target > motor2_current_pos) ? HIGH : LOW);

  while (motor1_current_pos != m1_target || motor2_current_pos != m2_target) {
    unsigned long now = micros();
    if (motor1_current_pos != m1_target && (now - m1_last_step_time >= step_delay)) {
      m1_last_step_time = now;
      digitalWrite(ST1, HIGH);
      delayMicroseconds(5);
      digitalWrite(ST1, LOW);
      motor1_current_pos += (m1_target > motor1_current_pos) ? 1 : -1;
    }
    if (motor2_current_pos != m2_target && (now - m2_last_step_time >= step_delay)) {
      m2_last_step_time = now;
      digitalWrite(ST2, HIGH);
      delayMicroseconds(5);
      digitalWrite(ST2, LOW);
      motor2_current_pos += (m2_target > motor2_current_pos) ? 1 : -1;
    }
  }
}