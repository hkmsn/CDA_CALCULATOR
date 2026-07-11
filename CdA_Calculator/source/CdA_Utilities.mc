import Toybox.Math;
import Toybox.Lang;
import Toybox.Activity;
import Toybox.Sensor;
// import Toybox.System; // Not used directly in Utilities module
import Toybox.SensorHistory;

module Utilities {
    // Constants
    const DEBUG            = false; // silences the garmin Data Table/updateDebugGarmin
    const BLE_DEBUG        = false;  // Enables verbose Bluetooth scanning and RX logs
    // Safe startup default: run from Garmin's internal sensors unless the
    // persisted "use_sensor" setting explicitly enables the BLE sensor.
    var   USING_SENSOR     = true;
    const TESTING_WITH_FAN = false;

    const AIR_SPEED_INX    = 0;
    const RHO_INX          = 1;
    const ALT_INX          = 2;
    const ALT_DIFF_INX     = 3;
    const EXPECTED_SENSOR_PARAMS = 4;  // Use to ensure missed BLE messages are not added.

    const ACCELERATION_GRAVITY   = 9.8067f;
    const IMPOSSIBLE_CLIMB_RATE  = 1.5f;  //1.5 m/s
    const DRIVE_TRAIN_EFFICIENCY = 0.025f; //ie. 97.75%
    const KE_JITTER_THRESHOLD    = 0.20f;
    const INERTIA_FACTOR         = 1.04f; //added mass to Bike + Ride weight
    const SPECIFIC_GAS_CONSTANT  = 287.058f;
    const ABSOLUTE_ZERO_TEMP     = 273.15f;
    const DEFAULT_AIR_DENSITY    = 1.225f; // Standard air density at sea level (kg/m^3)
	

    // Fusion Constants
    const ALT_FUSION_ALPHA       = 0.90f; // Weight for BMP/Inertial delta
    const ALT_FUSION_BETA        = 0.10f; // Weight for Garmin absolute/trend
    const ALT_SNAP_THRESHOLD     = 5.0f;  // Meters gap before snapping to Garmin

    function getStableTemp() as Float {
    // Check if the device supports SensorHistory
    if (Toybox has :SensorHistory && Toybox.SensorHistory has :getTemperatureHistory) {
        
        // Fetch the most recent 1 sample
        var sensorIter = SensorHistory.getTemperatureHistory({:period => 1, :order => SensorHistory.ORDER_NEWEST_FIRST});
        if (sensorIter != null) {
            var sample = sensorIter.next();
            if (sample instanceof SensorHistory.SensorSample && sample.data != null) {
                // Cast to Numeric to ensure toFloat() is available across all numeric types
                var data = sample.data as Lang.Numeric;
                return data.toFloat();
            }
        }
    }
    
    // Fallback if sensors aren't ready (e.g., just turned on the device)
    return 25.0f; 
}

    /**
     * Updates the debug array with raw Garmin sensor data.
     * @param debugArray The array to populate (usually _debugGarmin)
     * @param rawSpeed   Current speed from Activity.Info
     * @param rawAlt     Current altitude from Activity.Info
     * @param tempC      Not used Current temperature in Celsius
   
     */
    function updateDebugGarmin(debugArray as Array<Float?>, rawSpeed as Float, rawAlt as Float, tempC as Float, pressurePa as Float?) as Void {
        debugArray[AIR_SPEED_INX] = rawSpeed; // Garmin default: Air = Ground
        debugArray[ALT_INX]       = rawAlt;
        

        // Pre-calculate Garmin-only Rho for comparison in the debug table
        if (pressurePa != null) {
            debugArray[RHO_INX] = (pressurePa as Float) / (SPECIFIC_GAS_CONSTANT * (tempC + ABSOLUTE_ZERO_TEMP));
        } else {
            debugArray[RHO_INX] = null;
        }
    }

    /**
     * Calculates the filtered CHANGE in altitude (delta).
     * @param lastAlt         The absolute fused altitude from the PREVIOUS second.
     * @param deltaH_BMP      1s variation from BMP (meters). Can be null.
     * @param garminAltM      Current absolute Garmin Altitude (meters).
     * @param rollingGarminDeltaTrend   The smoothed altitude trend (meters) from Garmin.
     * @param accelZ          Optional vertical acceleration (m/s^2). Can be null.
     * @return                [Float delta, Float newAlt]
     */
    function getFilteredAltDelta(lastAlt as Float, deltaH_BMP as Float?, garminAltM as Float, rollingGarminDeltaDurSec as Float, accelZ as Float?) as Array<Float> {
        
        // 1. Fusion: If the high-fidelity BMP sensor is present, fuse its delta with the Garmin trend.
        // Otherwise, fallback to the Garmin trend exclusively.
        var primaryDelta = 0.0f;
        // Fix: 0.0 is a valid measurement (flat road). Only null means 'no data'.
        if (deltaH_BMP != null) {
            // Complementary filter: Alpha + Beta should ideally equal 1.0
            primaryDelta = (ALT_FUSION_ALPHA * deltaH_BMP) + (ALT_FUSION_BETA * rollingGarminDeltaDurSec);
        } else {
            primaryDelta = rollingGarminDeltaDurSec;
        }
        
        // 2. Optional Inertial Prediction (s = 0.5 * a * t^2)
        var inertialShift = (accelZ != null) ? (0.5f * (accelZ as Float)) : 0.0f;

        // 3. Predict the physical change
        var predictedDelta = (primaryDelta + inertialShift).toFloat();

        // 4. Calculate the NEW absolute altitude (for state tracking)
        // We fuse the prediction (lastAlt + change) with the absolute Garmin reference.
        var newAlt = (ALT_FUSION_ALPHA * (lastAlt + predictedDelta)) + (ALT_FUSION_BETA * garminAltM);

        // 5. SNAP Logic: If the gap is > 5m, snap to Garmin truth to prevent "fake climb power"
        var diff = newAlt - garminAltM;
        var absDiff = (diff < 0) ? -diff : diff;
        if (absDiff > ALT_SNAP_THRESHOLD) {
            newAlt = garminAltM;
        }

        // 6. Final sanitation for NaN before returning to the View
        if (newAlt != newAlt) { newAlt = lastAlt; }

        // 7. PHYSICS FIX: Use the predicted movement for the power model, 
        // NOT the state correction. This prevents the "Beta" correction 
        // (drift alignment) from being interpreted as a climb.
        return [predictedDelta.toFloat(), newAlt.toFloat()] as Array<Float>;
    }
}

class KalmanFilter {
    private var Q as Float; // Process noise covariance
    private var R as Float; // Measurement noise covariance
    private var P as Float; // Estimation error covariance
    private var K as Float = 0.0f; // Kalman gain
    private var x as Float?; // The estimated value (state)

    // q: Process noise (e.g., 0.1) - how much we trust the "model"
    // r: Measurement noise (e.g., 1.0) - how much we trust the sensor
    
    function initialize(q as Float, r as Float) {
        Q = q;
        R = r;
        P = 1.0f;
        x = null; // Initialized on first measurement
    }

    function update(measurement as Float) as Float {
        // 1. Initial measurement
        var currentX = x;
        if (currentX == null) {
            x = measurement;
            return measurement;
        }

        // 2. Prediction Update
        P = P + Q;

        // 3. Measurement Update (Correction)
        K = P / (P + R);
        
        var newX = currentX + K * (measurement - currentX);
        x = newX;
        P = (1.0f - K) * P;

        return newX;
    }
}

// This wrapper remains for backward compatibility with calls to $.getFilteredAltDelta
// but proxies the logic to the encapsulated module function.
function getFilteredAltDelta(lastAlt, deltaH_BMP, garminAltM, rollingGarminDeltaDurSec, accelZ) {
    return Utilities.getFilteredAltDelta(
        lastAlt as Float, 
        deltaH_BMP as Float?, 
        garminAltM as Float, 
        rollingGarminDeltaDurSec as Float, 
        (accelZ != null) ? (accelZ as Float) : null
    );
}
