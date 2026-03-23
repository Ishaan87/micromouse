#include "API.h"
#include "Config.h"
#include "Motors.h"
#include "Sensors.h"
#include "Encoders.h"

// ===========================================================================
// INTERNAL PID STATE
// Mirrors your original PID loop variables.
// These are reset at the start of each moveForward() / turn call.
// ===========================================================================
static float  s_currentHeading    = 0.0f;
static float  s_last_headingError = 0.0f;
static float  s_filtered_D_heading = 0.0f;
static float  s_last_positionError = 0.0f;
static float  s_filtered_D_position = 0.0f;
static unsigned long s_lastTimeMicros = 0;

// --- PID gains (copied from your original code) ---
static const float Kp_imu   = 3.5f;
static const float Kd_imu   = 0.2f;
static const float Kp_wall  = 0.4f;
static const float Kd_wall  = 0.05f;
static const float alpha_D  = 0.3f;
static const float Kp_dist_encoder = 0.5f;
static const float Kp_dist_lidar   = 2.0f;

static const int   IDEAL_SIDE_DIST = 45;
static const int   IDEAL_STOP_DIST = 45;

// ===========================================================================
// HELPERS
// ===========================================================================
static void resetPIDState() {
    s_currentHeading     = 0.0f;
    s_last_headingError  = 0.0f;
    s_filtered_D_heading = 0.0f;
    s_last_positionError = 0.0f;
    s_filtered_D_position= 0.0f;
    s_lastTimeMicros     = micros();
}

// One tick of the steering + speed PID — identical to your original loop().
// baseSpeed is set by the caller (move vs turn).
// Returns the correctionSpeed so the caller can apply it as needed.
static void pidTick(int baseSpeed) {
    unsigned long now = micros();
    float dt = max((now - s_lastTimeMicros) / 1000000.0f, 0.0001f);
    s_lastTimeMicros = now;

    // Integrate gyro → heading
    float gyroVel = readGyroHeading();
    s_currentHeading += gyroVel * dt;

    DistanceData lidars = readLidars();

    // --- Wall centering PD ---
    float positionError = 0.0f;
    bool hasLeft  = (lidars.left  < WALL_THRESHOLD_MM);
    bool hasRight = (lidars.right < WALL_THRESHOLD_MM);
    if      (hasLeft && hasRight)  positionError = lidars.left - lidars.right;
    else if (hasLeft  && !hasRight) positionError = lidars.left  - IDEAL_SIDE_DIST;
    else if (hasRight && !hasLeft)  positionError = IDEAL_SIDE_DIST - lidars.right;

    float raw_D_pos = (positionError - s_last_positionError) / dt;
    s_filtered_D_position = (alpha_D * raw_D_pos) + ((1.0f - alpha_D) * s_filtered_D_position);
    s_last_positionError = positionError;

    float targetHeading = (positionError * Kp_wall) + (s_filtered_D_position * Kd_wall);
    targetHeading = constrain(targetHeading, -10.0f, 10.0f);

    // --- IMU heading PD ---
    float headingError = targetHeading - s_currentHeading;
    float raw_D_head = (headingError - s_last_headingError) / dt;
    s_filtered_D_heading = (alpha_D * raw_D_head) + ((1.0f - alpha_D) * s_filtered_D_heading);
    s_last_headingError = headingError;

    int correction = (int)((Kp_imu * headingError) + (Kd_imu * s_filtered_D_heading));

    setMotorSpeeds(baseSpeed - correction, baseSpeed + correction);
}

// ===========================================================================
// API — MAZE SIZE
// ===========================================================================
int mazeWidth()  { return MAZE_WIDTH;  }
int mazeHeight() { return MAZE_HEIGHT; }

// ===========================================================================
// API — WALL SENSING
// Reads the three VL53L0X lidars.
// Returns true if a wall is present (reading below threshold).
// Must be called when the mouse is stationary at the centre of a cell.
// ===========================================================================
bool wallFront() {
    DistanceData d = readLidars();
    return d.front < WALL_THRESHOLD_MM;
}
bool wallLeft() {
    DistanceData d = readLidars();
    return d.left < WALL_THRESHOLD_MM;
}
bool wallRight() {
    DistanceData d = readLidars();
    return d.right < WALL_THRESHOLD_MM;
}

// ===========================================================================
// API — MOVE FORWARD ONE CELL
//
// Uses your existing two-loop PID strategy:
//   Loop A (translational): encoder count drives toward PULSES_PER_CELL.
//                           If front wall is close, lidar takes over.
//   Loop B (rotational):    IMU + wall centering keeps the mouse straight.
//
// BLOCKING — returns only after the mouse has travelled exactly one cell.
// ===========================================================================
void moveForward() {
    resetEncoders();
    resetPIDState();

    bool done = false;
    while (!done) {
        unsigned long now = micros();
        float dt = max((now - s_lastTimeMicros) / 1000000.0f, 0.0001f);
        s_lastTimeMicros = now;

        float gyroVel = readGyroHeading();
        s_currentHeading += gyroVel * dt;

        DistanceData lidars = readLidars();
        long currentTicks = (leftTicks + rightTicks) / 2;

        // --- Loop A: translational speed ---
        int   baseSpeed;
        float distError;
        if (lidars.front < WALL_THRESHOLD_MM) {
            distError = lidars.front - IDEAL_STOP_DIST;
            baseSpeed = (int)(distError * Kp_dist_lidar);
        } else {
            distError = (float)(PULSES_PER_CELL - currentTicks);
            baseSpeed = (int)(distError * Kp_dist_encoder);
        }
        baseSpeed = constrain(baseSpeed, -100, 150);

        // Stop condition: close enough to target distance
        if (abs(distError) < 5) {
            done = true;
        }

        // --- Loop B: steering (wall centering + IMU) ---
        float positionError = 0.0f;
        bool hasLeft  = (lidars.left  < WALL_THRESHOLD_MM);
        bool hasRight = (lidars.right < WALL_THRESHOLD_MM);
        if      (hasLeft && hasRight)   positionError = lidars.left - lidars.right;
        else if (hasLeft  && !hasRight) positionError = lidars.left  - IDEAL_SIDE_DIST;
        else if (hasRight && !hasLeft)  positionError = IDEAL_SIDE_DIST - lidars.right;

        float raw_D_pos = (positionError - s_last_positionError) / dt;
        s_filtered_D_position = (alpha_D * raw_D_pos) + ((1.0f - alpha_D) * s_filtered_D_position);
        s_last_positionError = positionError;

        float targetHeading = (positionError * Kp_wall) + (s_filtered_D_position * Kd_wall);
        targetHeading = constrain(targetHeading, -10.0f, 10.0f);

        float headingError = targetHeading - s_currentHeading;
        float raw_D_head = (headingError - s_last_headingError) / dt;
        s_filtered_D_heading = (alpha_D * raw_D_head) + ((1.0f - alpha_D) * s_filtered_D_heading);
        s_last_headingError = headingError;

        int correction = (int)((Kp_imu * headingError) + (Kd_imu * s_filtered_D_heading));

        setMotorSpeeds(baseSpeed - correction, baseSpeed + correction);
    }

    stopMotors();
    delay(50);   // brief settle before next sensor read
}

// ===========================================================================
// API — TURN LEFT 90°
//
// Point turn: left motor backward, right motor forward.
// IMU (gyro integration) is the reference — stops when heading change
// reaches TARGET_TURN_DEG within TURN_TOLERANCE degrees.
// BLOCKING.
// ===========================================================================
void turnLeft() {
    resetPIDState();
    float turnedDeg = 0.0f;
    unsigned long lastT = micros();

    while (turnedDeg < (TARGET_TURN_DEG - TURN_TOLERANCE)) {
        unsigned long now = micros();
        float dt = max((now - lastT) / 1000000.0f, 0.0001f);
        lastT = now;

        // Integrate gyro for this turn.
        // For a left turn the gyro reads negative on the vertical axis
        // (depending on mount orientation). Take the absolute value so
        // turnedDeg always increases. If your turns go the wrong way,
        // swap the motor directions below.
        float vel = readGyroHeading();
        turnedDeg += fabsf(vel) * dt;

        // Proportional slow-down as we approach target — avoids overshoot
        float remaining = TARGET_TURN_DEG - turnedDeg;
        int spd = (int)constrain(remaining * 2.0f, 40, TURN_BASE_SPEED);

        // Left turn: left wheel backward, right wheel forward
        setMotorSpeeds(-spd, spd);
    }

    stopMotors();
    delay(50);
}

// ===========================================================================
// API — TURN RIGHT 90°
// Mirror of turnLeft — right wheel backward, left wheel forward.
// BLOCKING.
// ===========================================================================
void turnRight() {
    resetPIDState();
    float turnedDeg = 0.0f;
    unsigned long lastT = micros();

    while (turnedDeg < (TARGET_TURN_DEG - TURN_TOLERANCE)) {
        unsigned long now = micros();
        float dt = max((now - lastT) / 1000000.0f, 0.0001f);
        lastT = now;

        float vel = readGyroHeading();
        turnedDeg += fabsf(vel) * dt;

        float remaining = TARGET_TURN_DEG - turnedDeg;
        int spd = (int)constrain(remaining * 2.0f, 40, TURN_BASE_SPEED);

        // Right turn: right wheel backward, left wheel forward
        setMotorSpeeds(spd, -spd);
    }

    stopMotors();
    delay(50);
}

// ===========================================================================
// API — RESET BUTTON
// wasReset() returns true while the button is held (active LOW).
// ackReset() does nothing on hardware — just a stub for API compatibility.
// ===========================================================================
bool wasReset() {
    return digitalRead(RESET_BUTTON_PIN) == LOW;
}

void ackReset() {
    // No-op on hardware. In MMS this sends an ack over the pipe.
    // On hardware the button state clears itself when released.
}

// ===========================================================================
// API — HARDWARE INIT
// Call this once from setup() before starting the algorithm.
// ===========================================================================
void initAPI() {
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    initMotors();
    initEncoders();
    initSensors();
    calibrateGyro();
}