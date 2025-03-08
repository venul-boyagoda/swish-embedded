#include <stdint.h> // Include the header for int32_t
#include <stdbool.h> // Include the header for bool
#include <math.h>    // Include the header for math functions like fabsf

// Mahony filter variables and functions
#define twoKpDef    (2.0f * 0.5f)    // 2 * proportional gain (reduced from 0.5f to 0.03f)
#define twoKiDef    (2.0f * 0.01f)     // 2 * integral gain

volatile float twoKp = twoKpDef;        // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;        // 2 * integral gain (Ki)
volatile float qq0 = 1.0f, qq1 = 0.0f, qq2 = 0.0f, qq3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

static float invSqrt(float x) {
    union {
        float f;
        int32_t i;
    } y;
    float halfx = 0.5f * x;

    y.f = x;
    y.i = 0x5f375a86 - (y.i >> 1);
    y.f = y.f * (1.5f - (halfx * y.f * y.f));
    y.f = y.f * (1.5f - (halfx * y.f * y.f));
    y.f = y.f * (1.5f - (halfx * y.f * y.f));
    return y.f;
}

// Add the function to detect significant motion
bool isSignificantMotion(float ax, float ay, float az, float gx, float gy, float gz) {
    // Check if gyro readings exceed threshold (in rad/s)
    const float GYRO_THRESHOLD = 0.05f; // 0.02 originally
    if (fabsf(gx) > GYRO_THRESHOLD || fabsf(gy) > GYRO_THRESHOLD || fabsf(gz) > GYRO_THRESHOLD) {
        return 1;
    }
    
    // Check if acceleration vector is significantly different from gravity
    const float ACCEL_THRESHOLD = 0.07f; // 0.05 originally 0.03 other option
    float accelMagnitude = sqrtf(ax*ax + ay*ay); // removed az from magnitude calculations
    if (fabsf(accelMagnitude) > ACCEL_THRESHOLD) { // Assuming normalized accel "removed -1.0f"
        return 1;
    }
    
    return 0;
}

// Add gain adjustment function to dynamically adjust sensitivity based on motion
void adjustGains(float gx, float gy, float gz) {
    // Calculate total angular velocity magnitude
    float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);
    
    // Adjust gains based on motion intensity
    if (gyroMag > 0.5f) {
        // Higher gains during faster movements
        twoKp = 2.0f * 0.25f;  // More aggressive tracking during motion
    } else if (gyroMag > 0.1f) {
        // Medium gains during moderate movements
        twoKp = 2.0f * 0.15f;  // Default value
    } else {
        // Lower gains during small movements to prevent jitter
        twoKp = 2.0f * 0.08f;
    }
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    adjustGains(gx, gy, gz);

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = qq1 * qq3 - qq0 * qq2;
        halfvy = qq0 * qq1 + qq2 * qq3;
        halfvz = qq0 * qq0 - 0.5f + qq3 * qq3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // SOLUTION 3: Skip accelerometer correction if there's very little motion
        if (!isSignificantMotion(ax, ay, az, gx, gy, gz)) {
            // Set error terms to zero when there's minimal motion
            halfex = 0.0f;
            halfey = 0.0f;
            halfez = 0.0f;
        }

        // SOLUTION 4: Check for unreasonable corrections
        const float MAX_CORRECTION_MAGNITUDE = 0.5f;
        float correctionMagnitude = sqrtf(halfex*halfex + halfey*halfey + halfez*halfez);
        if (correctionMagnitude > MAX_CORRECTION_MAGNITUDE) {
            // Correction is too large, scale it down
            float scaleFactor = MAX_CORRECTION_MAGNITUDE / correctionMagnitude;
            halfex *= scaleFactor;
            halfey *= scaleFactor;
            halfez *= scaleFactor;
        }

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            // integral error scaled by Ki
            integralFBx += twoKi * halfex * (1.0f / 100.0f);
            integralFBy += twoKi * halfey * (1.0f / 100.0f);
            integralFBz += twoKi * halfez * (1.0f / 100.0f);
            gx += integralFBx;    // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;    // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / 100.0f));        // pre-multiply common factors
    gy *= (0.5f * (1.0f / 100.0f));
    gz *= (0.5f * (1.0f / 100.0f));
    qa = qq0;
    qb = qq1;
    qc = qq2;
    qq0 += (-qb * gx - qc * gy - qq3 * gz);
    qq1 += (qa * gx + qc * gz - qq3 * gy);
    qq2 += (qa * gy - qb * gz + qq3 * gx);
    qq3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(qq0 * qq0 + qq1 * qq1 + qq2 * qq2 + qq3 * qq3);
    qq0 *= recipNorm;
    qq1 *= recipNorm;
    qq2 *= recipNorm;
    qq3 *= recipNorm;
}