// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <MotorCtrl.hpp>
#include <Wire.h>
#include <protocentral_TLA20xx.h>

// Include Bluepad32 allowlist functions
extern "C" {
    #include "bt/uni_bt_allowlist.h"
}

// Motor Control Configuration
#define MOTOR_ID 1
#define CAN_TX_PIN GPIO_NUM_1
#define CAN_RX_PIN GPIO_NUM_2
#define MOTOR_SWITCH GPIO_NUM_38

#define MOTOR_OFFSET -1.0471975512  // Motor offset for position correction

// ADC / Hall Sensor Configuration
#define TLA20XX_I2C_ADDR 0x48  // Default I2C address for TLA2024
#define SDA GPIO_NUM_8        // I2C SDA pin
#define SCL GPIO_NUM_7        // I2C SCL pin
#define ADC_CHANNEL_VOLTAGE 0
#define ADC_CHANNEL_CURRENT 1
#define ADC_CHANNEL_HALL 2

// Motor Control Variables
Motor motor;
ButterworthFilter positionFilter(20, 100);  // 20Hz cutoff, 100Hz sampling rate
Motor_state motorState;
bool motorRunning = false;
bool motorCalibrated = false;
float targetPosition = 0.0;
float targetVelocity = 0.0;
float commandKp = 8.0;
float commandKd = 0.2;
bool enableFilter = false; // true;

// Motor Crash Detection Variables
float lastTargetPosition = 0.0;
float lastMotorAngle = 0.0;
unsigned long lastAngleChangeTime = 0;
unsigned long crashDetectionStartTime = 0;
const unsigned long CRASH_DETECTION_TIMEOUT = 2000;  // 2 seconds
const float POSITION_CHANGE_THRESHOLD = 0.05;  // Minimum position change to consider movement
const float ANGLE_CHANGE_THRESHOLD = 0.02;     // Minimum angle change to detect motor movement
const float TORQUE_ABNORMAL_THRESHOLD = 25.0;  // Maximum normal torque
const float TEMPERATURE_LIMIT = 80.0;          // Temperature warning limit

// Hall sensor / ADC monitoring
TLA20XX tla2024(TLA20XX_I2C_ADDR);
float monitoredValue = 0.0;
uint8_t currentADCChannel = ADC_CHANNEL_HALL;

// ADC Helper Functions
void setADCChannel(uint8_t channel) {
    if (channel == ADC_CHANNEL_VOLTAGE) {
        tla2024.setMux(TLA20XX::MUX_AIN0_GND);
    } else if (channel == ADC_CHANNEL_CURRENT) {
        tla2024.setMux(TLA20XX::MUX_AIN1_GND);
    } else if (channel == ADC_CHANNEL_HALL) {
        tla2024.setMux(TLA20XX::MUX_AIN2_GND);
    }
    delay(10);  // Wait for channel switch
}

void initADC() {
    Console.println("Initializing TLA2024 ADC...");
    Wire.setPins(SDA, SCL);
    Wire.begin();
    delay(100);
    
    tla2024.begin();
    tla2024.setMode(TLA20XX::OP_CONTINUOUS);
    tla2024.setDR(TLA20XX::DR_3300SPS);
    tla2024.setFSR(TLA20XX::FSR_4_096V);
    
    // Set default channel to hall sensor
    setADCChannel(ADC_CHANNEL_HALL);
    delay(100);
    
    Console.println("ADC initialized");
}

void updateMonitoredValue() {
    monitoredValue = tla2024.read_adc();
}

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default, it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Motor Control Helper Functions
void initMotor() {
    Console.println("Initializing motor...");
    motor.Init(MOTOR_ID, CAN_TX_PIN, CAN_RX_PIN);
    motor.Set_mode(Motor_mode::Motion);
    motor.Set_parameter(Motor_param::limit_cur, 27.0F);
    Console.println("Motor initialized");
}

void enableMotor() {
    if (!motorRunning) {
        Console.println("Enabling motor...");
        motorState = motor.Enable();
        motorRunning = true;
        Console.println("Motor enabled");
    }
}

void disableMotor() {
    if (motorRunning) {
        Console.println("Disabling motor...");
        motorState = motor.Disable();
        motorRunning = false;
        Console.println("Motor disabled");
    }
}

bool checkMotorSafety() {
    if (!motorCalibrated) {
        Console.println("Motor not calibrated!");
        return false;
    }
    return true;
}

void moveToMiddle() {
    Console.println("Moving to middle position...");
    enableMotor();
    for (float i = 0; i <= PI/2; i += 0.01) {
        float filteredPos = -MOTOR_OFFSET * cos(i);
        motorState = motor.Set_control(0, MOTOR_OFFSET+filteredPos, 0, 20, 0.5);
        delay(10);
    }
    Console.println("Reached middle position");
}

// Spin motor until threshold is reached (for calibration)
float spinUntilThreshold(float targetValue, float speed, bool reverse = false) {
    float motorAngleSum = 0.0F;
    float motorTempVal = 0.0F;
    
    motorState = motor.Set_control(0, 0, reverse ? -speed : speed, 0, 40);
    
    Console.printf("Spinning %s, waiting for hall sensor >= %.1f...\n", 
                   reverse ? "reverse" : "forward", targetValue);
    
    // Wait until monitored_value is below the target threshold
    while (monitoredValue < targetValue) {
        delay(1);
        updateMonitoredValue();
        motorTempVal = motor.Get_state().angle;
    }
    motorAngleSum += motorTempVal;
    Console.printf("Hall sensor reached %.1f, angle: %.3f\n", monitoredValue, motorTempVal);

    // Wait until monitored_value exceeds a secondary threshold for finer control
    while (monitoredValue > targetValue - 20) {
        delay(1);
        updateMonitoredValue();
        motorTempVal = motor.Get_state().angle;
    }
    motorAngleSum += motorTempVal;
    Console.printf("Hall sensor stabilized at %.1f, angle: %.3f\n", monitoredValue, motorTempVal);
    
    return motorAngleSum;
}

// Main function to align motor and set zero position using hall sensor
void alignMotorAndSetZero(Motor &motor, float hall_threshold, float speed = 0.5F) {
    Console.printf("Aligning motor with hall threshold: %.1f, speed: %.2f\n", hall_threshold, speed);
    float motor_total = 0.0F;

    // Spin motor in the forward direction
    Console.println("Spinning forward...");
    motor_total += spinUntilThreshold(hall_threshold, speed);

    // Pause before reversing
    delay(500);

    // Spin motor in the reverse direction
    Console.println("Spinning reverse...");
    motor_total += spinUntilThreshold(hall_threshold, speed, true);

    // Average the total motor angle and set position
    Console.printf("Setting position to averaged angle: %.3f\n", motor_total / 4.0F);
    motorState = motor.Set_control(0, motor_total / 4.0F, 0, 20, 0.5);
    delay(1000);

    // Disable motor and set zero position
    disableMotor();
    motorState = motor.Set_zero();
    Console.println("Motor aligned and zero position set");
}

void autoCalibrate() {
    Console.println("Starting auto-calibration...");
    
    disableMotor();
    delay(100);
    
    // Set ADC to hall sensor channel
    setADCChannel(ADC_CHANNEL_HALL);
    delay(100);
    
    // Check initial hall sensor reading
    updateMonitoredValue();
    Console.printf("Initial hall sensor reading: %.1f\n", monitoredValue);
    
    // Reinitialize motor
    initMotor();
    motorState = motor.Set_zero();
    delay(100);
    
    enableMotor();
    
    // Align motor using hall sensor threshold
    alignMotorAndSetZero(motor, 940, 0.5);

    delay(100);
    
    motorCalibrated = true;
    Console.println("Calibration complete!");
    
    // Switch back to voltage monitoring
    setADCChannel(ADC_CHANNEL_VOLTAGE);
    
    // Move to middle position if offset is set
    if (MOTOR_OFFSET != 0) {
        moveToMiddle();
    }
}

void stepMotor(float targetAngle, float targetVel, float kp, float kd) {
    if (checkMotorSafety()) {
        motorState = motor.Set_control(0, targetAngle + MOTOR_OFFSET, targetVel, kp, kd);
    }
    if (((motorState.error_state >> 6) & 0x03) == 0) {
        digitalWrite(MOTOR_SWITCH, HIGH); // Turn off motor switch on critical error
        delay(500);
        esp_restart();
    }
}

// Motor Crash Detection Function
bool detectMotorCrash() {
    // Get current motor state
    motorState = motor.Get_state();
    
    // Check 1: Temperature limit exceeded
    if (motorState.temperature > TEMPERATURE_LIMIT) {
        Console.printf("CRASH DETECTED: Temperature too high (%.1f°C)\n", motorState.temperature);
        return true;
    }
    
    // Check 2: Torque abnormally high (motor stalled/blocked)
    if (abs(motorState.torque) > TORQUE_ABNORMAL_THRESHOLD) {
        Console.printf("CRASH DETECTED: Abnormal torque (%.2f)\n", motorState.torque);
        return true;
    }
    
    // Check 3: Target position is changing but motor angle is not responding
    float targetChange = abs(targetPosition - lastTargetPosition);
    float angleChange = abs(motorState.angle - lastMotorAngle);
    
    // If target position changed significantly
    if (targetChange > POSITION_CHANGE_THRESHOLD) {
        if (crashDetectionStartTime == 0) {
            // Start tracking
            crashDetectionStartTime = millis();
        }
        
        // Check if motor angle changed
        if (angleChange > ANGLE_CHANGE_THRESHOLD) {
            // Motor is responding, reset detection
            crashDetectionStartTime = 0;
            lastAngleChangeTime = millis();
        } else {
            // Motor not responding, check timeout
            if (millis() - crashDetectionStartTime > CRASH_DETECTION_TIMEOUT) {
                Console.printf("CRASH DETECTED: Target changed (%.3f -> %.3f) but motor stuck at %.3f\n",
                             lastTargetPosition, targetPosition, motorState.angle);
                return true;
            }
        }
    }
    // Note: Don't reset timer when target stops - keep monitoring until motor responds
    
    // Update last values for next iteration
    lastTargetPosition = targetPosition;
    lastMotorAngle = motorState.angle;
    
    return false;
}

void handleMotorCrash() {
    Console.println("====================================");
    Console.println("MOTOR CRASH DETECTED - RESTARTING!");
    Console.println("====================================");
    // delay(1000);  // Give time to print message
    digitalWrite(MOTOR_SWITCH, HIGH); // Turn off motor switch on critical error
    delay(500);
    esp_restart();
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void dumpMouse(ControllerPtr ctl) {
    Console.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    static const char* key_names[] = {
        // clang-format off
        // To avoid having too much noise in this file, only a few keys are mapped to strings.
        // Starts with "A", which is offset 4.
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
        // Special keys
        "Enter", "Escape", "Backspace", "Tab", "Spacebar", "Underscore", "Equal", "OpenBracket", "CloseBracket",
        "Backslash", "Tilde", "SemiColon", "Quote", "GraveAccent", "Comma", "Dot", "Slash", "CapsLock",
        // Function keys
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
        // Cursors and others
        "PrintScreen", "ScrollLock", "Pause", "Insert", "Home", "PageUp", "Delete", "End", "PageDown",
        "RightArrow", "LeftArrow", "DownArrow", "UpArrow",
        // clang-format on
    };
    static const char* modifier_names[] = {
        // clang-format off
        // From 0xe0 to 0xe7
        "Left Control", "Left Shift", "Left Alt", "Left Meta",
        "Right Control", "Right Shift", "Right Alt", "Right Meta",
        // clang-format on
    };
    Console.printf("idx=%d, Pressed keys: ", ctl->index());
    for (int key = Keyboard_A; key <= Keyboard_UpArrow; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = key_names[key - 4];
            Console.printf("%s,", keyName);
        }
    }
    for (int key = Keyboard_LeftControl; key <= Keyboard_RightMeta; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = modifier_names[key - 0xe0];
            Console.printf("%s,", keyName);
        }
    }
    Console.printf("\n");
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Console.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}

void processGamepad(ControllerPtr ctl) {
    // Motor control with gamepad buttons
    
    // A button: Toggle motor on/off
    if (ctl->a()) {
        if (motorRunning) {
            disableMotor();
        } else {
            enableMotor();
        }
        delay(300);  // Debounce
    }

    // B button: Run calibration
    if (ctl->b()) {
        Console.println("Calibration requested via gamepad");
        autoCalibrate();
        delay(300);  // Debounce
    }

    // X button: Set current position as zero
    if (ctl->x()) {
        Console.println("Setting zero position");
        motorState = motor.Set_zero();
        motorCalibrated = true;
        delay(300);  // Debounce
    }

    // Y button: Move to middle position
    if (ctl->y()) {
        if (motorCalibrated) {
            moveToMiddle();
        }
        delay(300);  // Debounce
    }

    // Use left joystick for coarse position control and right joystick for fine-tuning
    if (motorRunning && motorCalibrated) {
        // Map left joystick X axis (-511 to 512) to position (-PI to PI)
        int axisX = ctl->axisX();
        int axisY = ctl->axisY();
        int axisRX = ctl->axisRX();  // Right joystick X for fine-tuning
        
        float coarsePosition = 0;
        float finePosition = 0;
        
        // Add deadzone for left joystick (coarse control)
        if (abs(axisX) > 50) {
            coarsePosition = (float)axisX / 512.0 * PI;  // Full range: -PI to PI
        }
        
        // Add deadzone for right joystick (fine control - 1/10th range)
        if (abs(axisRX) > 50) {
            finePosition = (float)axisRX / 512.0 * PI * 0.2;  // Fine range: -PI/10 to PI/10
        }
        
        // Combine coarse and fine positions
        targetPosition = coarsePosition + finePosition;
        
        // Optional: Use left joystick Y for velocity (currently disabled)
        // targetVelocity = -(float)axisY / 512.0 * 10.0;  // Max 10 rad/s
        targetVelocity = 0;
        
        // Only print if either joystick is moved
        if (abs(axisX) > 50 || abs(axisRX) > 50) {
            Console.printf("Coarse: %.2f, Fine: %.2f, Total: %.2f\n", 
                         coarsePosition, finePosition, targetPosition);
        }
        
        // // Use R1/R2 to adjust Kp
        // if (ctl->r1()) {
        //     commandKp += 0.5;
        //     Console.printf("Kp increased to: %.1f\n", commandKp);
        //     delay(100);
        // }
        // if (ctl->r2()) {
        //     commandKp = max(0.0f, commandKp - 0.5);
        //     Console.printf("Kp decreased to: %.1f\n", commandKp);
        //     delay(100);
        // }
        
        // // Use L1/L2 to adjust Kd
        // if (ctl->l1()) {
        //     commandKd += 0.1;
        //     Console.printf("Kd increased to: %.2f\n", commandKd);
        //     delay(100);
        // }
        // if (ctl->l2()) {
        //     commandKd = max(0.0f, commandKd - 0.1);
        //     Console.printf("Kd decreased to: %.2f\n", commandKd);
        //     delay(100);
        // }
    }

    // Optional: still show gamepad data
    // dumpGamepad(ctl);
}

void processMouse(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->scrollWheel() > 0) {
        // Do Something
    } else if (ctl->scrollWheel() < 0) {
        // Do something else
    }

    // See "dumpMouse" for possible things to query.
    dumpMouse(ctl);
}

void processKeyboard(ControllerPtr ctl) {
    if (!ctl->isAnyKeyPressed())
        return;

    // This is just an example.
    if (ctl->isKeyPressed(Keyboard_A)) {
        // Do Something
        Console.println("Key 'A' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftShift)) {
        // Do something else
        Console.println("Key 'LEFT SHIFT' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        // Do something else
        Console.println("Key 'Left Arrow' pressed");
    }

    // See "dumpKeyboard" for possible things to query.
    dumpKeyboard(ctl);
}

void processBalanceBoard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->topLeft() > 10000) {
        // Do Something
    }

    // See "dumpBalanceBoard" for possible things to query.
    dumpBalanceBoard(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
            } else {
                Console.printf("Unsupported controller\n");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    set_led_color(240, 255, 0);

    pinMode(MOTOR_SWITCH, OUTPUT);
    digitalWrite(MOTOR_SWITCH, LOW); // Turn on motor


    // Setup the Bluepad32 callbacks, and the default behavior for scanning or not.
    // By default, if the "startScanning" parameter is not passed, it will do the "start scanning".
    // Notice that "Start scanning" will try to auto-connect to devices that are compatible with Bluepad32.
    // E.g: if a Gamepad, keyboard or mouse are detected, it will try to auto connect to them.
    bool startScanning = true;
    BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);

    // Notice that scanning can be stopped / started at any time by calling:
    // BP32.enableNewBluetoothConnections(enabled);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // ===== CONFIGURE ALLOWLIST TO ONLY CONNECT TO SPECIFIC CONTROLLER =====
    // Your controller's MAC address: 40:8E:2C:6C:EC:CF
    bd_addr_t allowed_controller = {0x40, 0x8E, 0x2C, 0x6C, 0xEC, 0xCF};
    
    // Clear any existing allowlist entries
    uni_bt_allowlist_remove_all();
    
    // Add your controller to the allowlist
    uni_bt_allowlist_add_addr(allowed_controller);
    
    // Enable the allowlist feature
    uni_bt_allowlist_set_enabled(true);
    
    Console.println("Bluetooth allowlist enabled - only controller 40:8E:2C:6C:EC:CF can connect");
    
    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(false);

    // ===== MOTOR INITIALIZATION =====
    Console.println("\n===== Starting Motor Initialization =====");
    delay(1000);  // Wait for system to stabilize
    
    // Initialize ADC for hall sensor
    initADC();
    delay(500);
    
    // Initialize motor
    initMotor();
    delay(500);
    
    // Run auto-calibration
    Console.println("Starting auto-calibration sequence...");
    autoCalibrate();
    
    Console.println("===== Motor Setup Complete =====");
    Console.println("Control the motor with your gamepad:");
    Console.println("  A: Toggle motor on/off");
    Console.println("  B: Run calibration");
    Console.println("  X: Set current position as zero");
    Console.println("  Y: Move to middle position");
    Console.println("  Left Stick X: Coarse position control (-PI to PI)");
    Console.println("  Right Stick X: Fine position tuning (-PI/10 to PI/10)");
    Console.println("  R1/R2: Adjust Kp gain");
    Console.println("  L1/L2: Adjust Kd gain");
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // Update monitored value from ADC
    updateMonitoredValue();
    
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // Update motor with filtered position if motor is running
    if (motorRunning && motorCalibrated) {
        float filteredPos = enableFilter ? positionFilter.filter(targetPosition) : targetPosition;
        stepMotor(filteredPos, targetVelocity, commandKp, commandKd);
        
        // Check for motor crash conditions
        if (detectMotorCrash()) {
            handleMotorCrash();
        }
        
        // Get and display motor state periodically
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 500) {  // Print every 500ms
            motorState = motor.Get_state();
            Console.printf("Motor - Pos: %.3f, Vel: %.3f, Torque: %.3f, Temp: %.1f, Hall: %.1f\n", 
                         motorState.angle, motorState.angle_v, motorState.torque, 
                         motorState.temperature, monitoredValue);
            lastPrint = millis();
        }
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    delay(20);  // 50Hz update rate
}
