import Toybox.Application;
import Toybox.Sensor;
import Toybox.FitContributor;
import Toybox.Lang;
import Toybox.WatchUi;
import Toybox.Graphics;
import Toybox.UserProfile;
//import Toybox.AntPlus;
import Toybox.System;
import Toybox.Activity;
import Toybox.Math;
import Utilities;

class CdA_CalculatorView extends WatchUi.DataField { 
	// KEY_PRESS constant isn't available on older devices (e.g. Edge540).
	// We'll respond to KEY_DOWN events instead and drop the alias.

    private var _altKalman as KalmanFilter?;
    private var _emaAlpha as Float = 0.15f;
    private var _bleDelegate as Lang.Object? = null;
    private var _bleRetryTick as Number = 0;
    private var _sensorWatchdog as Number = 0;

	private var _cdaField as FitContributor.Field?;

	var DF_Title_Text_1 = "";
	var DF_Title_Text_2 = "";
	var DF_Title_Text_3 = "";
	var DF_Title_Text_4 = "";

	var DF_Title_x = 10;
	var DF_Title_y = 10;
	var DF_Title_font = Graphics.FONT_XTINY;

	var aeroPower     as Float = 0.0f;
	var powerAvg      as Float = 0.0f;
	var CdAValue      as Float = 0.0f;
	var climbPowerUsage    as Float = 0.0f;
	var frictionPowerUsage as Float = 0.0f;
	var kineticEnergyVariationPowerUsage as Float = 0.0f;
	var driveTrainPowerUsage as Float = 0.0f;
	var netPowerNoCdA as Float = 0.0f;
	var avgDragFactor as Float = 0.0f;
	var bikeWeight     = 8.5f; // Weight in Kilograms
	var crrValue       = 0.0045f; // Standard Crr for average asphalt
    var staticWeight   = 90.0f; // Static mass (rider + bike) in kg
    var effectiveMass  = 93.6f; // Static mass * inertia factor for acceleration

	var Value_x = 0;
	var Value_y = 0;
	var Value_font       = Graphics.FONT_XTINY;
	var Row_Value_font   = Graphics.FONT_XTINY;
	var Speed_Value_font = Graphics.FONT_XTINY;
	var CdA_Value_font   = Graphics.FONT_XTINY;

	var Value_Unit = "XX";
	var Unit_x     = 0;
	var Unit_y     = 0;
	var Unit_font  = Graphics.FONT_XTINY;

	var power as Float = 0.0f;
	var waitReason as String = "Wait";
	var sensorParams as Array<Float?>?;
	var page = 0;

	var history_speedMSec   as Array<Float>?;
	var history_speedSqDiff as Array<Float>?;
	var history_power       as Array<Float>?;
	var history_speedAirSensorMSec   as Array<Float>?;
	var history_garmin_altitude_diff as Array<Float>?;
	var history_dragPowerFactor      as Array<Float>?;
	 
	var speedMSec 		  as Float = 0.0f;
	var speedMSecPrevious as Float = 0.0f;
    var speedAvgSqDiff    as Float = 0.0f;
	var speedAvgMSec      as Float = 0.0f;
	var speedAvgKmHr      as Number = 0;
	var speedAvgAirSensorMSec = 0.0f;
	var speedAirSensorMSec as Float = 0.0f;
    var currentSpeedSqDiff as Float = 0.0f;
    
    var garminAvgAltitudeChange     as  Float = 0.0f;
    var altitude                    as  Float = 0.0f;
    var avgVerticalSpeedMS          as  Float = 0.0f; // Average vertical rate of change in m/s
    private var _hasValidAltitude   as Boolean = false;
    private var _garminAltPrevious  as Float = 0.0f;
	var altitudePrevious    as Float = 0.0f;
	var garminAltDelta      as Float = 0.0f;
	var	sumOfSpeedMSec      as Float = 0.0f;
	var sumOfSpeedSqDiff    as Float = 0.0f;

    var sumOfSpeedAirSensorMSec as Float = 0.0f;
	var	sumOfPower              as Float = 0.0f;
	var sumOfDragPowerFactor    as Float = 0.0f;
	var sumOfFusedAltitudeDifferences as Float = 0.0f;
    var samplesCollected as Number = 0;
    
	var duration as Number = 3; // Default minimum duration for stable CdA
	var durationIdxNext    = 0;  
    var airDensity as Float = Utilities.DEFAULT_AIR_DENSITY;
    //! Set the label of the data field here.

    function initialize(params as Array<Lang.Any>?) {   
        WatchUi.DataField.initialize();

        // Load user-defined parameters from settings (Crr, weight, duration)
        updateSettings();

        _altKalman = new $.KalmanFilter(0.01f, 1.0f);

        // Setup initial history buffers based on loaded duration
        setupHistoryBuffers();

        _cdaField = createField(
			"CdA", 0, FitContributor.DATA_TYPE_FLOAT,
			{ :mesgType => FitContributor.MESG_TYPE_RECORD, :units => "m^2" }
		);
    }

    // Called by BLE Delegate to notify that external sensor data has arrived
    function notifySensorUpdate() as Void {
        _sensorWatchdog = 0; // Reset watchdog whenever valid data arrives
    }

    // Set by the BLE Delegate to allow compute() to trigger retries
    function setBleDelegate(delegate as Lang.Object) as Void {
        _bleDelegate = delegate;
    }

    //! The given info object contains all the current workout
    //! information. Calculate a value and return it in this method.

    function compute(info as Activity.Info) as Numeric? {
    // 1. Local data capture and basic validation
        var sParams      = sensorParams;
        var curSpeed     = info.currentSpeed;
        var curPower     = info.currentPower;
        var garminCurAlt = info.altitude;

        // Detect NaN or Null - simulator often starts with null or NaN during transitions
        var rawSpeed = (curSpeed != null && curSpeed == curSpeed) ? (curSpeed as Float) : 0.0f;
        var hasGarminAlt = (garminCurAlt != null && garminCurAlt == garminCurAlt);
        var rawAlt   = hasGarminAlt ? (garminCurAlt as Float) : altitude;
        speedMSec = rawSpeed; // Update immediately so the UI reflects speed even below the calculation threshold
        power = (curPower != null && curPower == curPower) ? (curPower as Number).toFloat() : 0.0f;

        // Guard: If settings changed to disable sensor, ensure we don't try to use stale params
        if (!Utilities.USING_SENSOR) {
            sensorParams = null;
            sParams = null;
            _sensorWatchdog = 0;
        }

        // Watchdog: Reset to 0 by notifySensorUpdate() in the delegate when real data arrives.
        if (Utilities.USING_SENSOR) {
            _sensorWatchdog++;
        }

        if (Utilities.USING_SENSOR && _sensorWatchdog > 5) {
            if (sensorParams != null) {
                if (Utilities.BLE_DEBUG) { System.println("BLE: Data timeout (5s). Falling back to GPS."); }
                sensorParams = null;
            }
            
            // If no data for 15 seconds, force the BLE stack to restart scanning entirely
            if (_sensorWatchdog == 15 && _bleDelegate != null && _bleDelegate has :resetConnection) {
                _bleDelegate.resetConnection();
            }
        }

        var tempC      = Utilities.getStableTemp(); // Garmin internal or fallback
        var pressurePa = info.ambientPressure;      // Garmin internal
        if (pressurePa != null && pressurePa != pressurePa) { pressurePa = null; }

        // Use the 1Hz compute loop as a safe "Timer" to retry BLE handshake if needed
        if (Utilities.USING_SENSOR) {
            _bleRetryTick++;
            if (_bleRetryTick >= 2) { // Slightly faster retry (2s) helps resolve stalls quicker on Edge 540
                _bleRetryTick = 0;
                if (_bleDelegate != null && _bleDelegate has :attemptRetry) {
                    _bleDelegate.attemptRetry();
                }
            }
        } else {
            _bleRetryTick = 0;
        }

        // Determine Air Density: Priority BLE Sensor -> Fallback Garmin Internal
        var sRho = (sParams != null && sParams.size() > Utilities.RHO_INX) ? sParams[Utilities.RHO_INX] : null;
        if (sRho != null && sRho > 0.1f) {
            airDensity = sRho.toFloat();
        } else if (pressurePa != null && pressurePa > 30000.0f) {
            var calcRho = (pressurePa as Float) / (Utilities.SPECIFIC_GAS_CONSTANT * (tempC + Utilities.ABSOLUTE_ZERO_TEMP));
            airDensity = (calcRho > 0.5f && calcRho < 1.5f) ? calcRho : Utilities.DEFAULT_AIR_DENSITY;
        } else {
            airDensity = Utilities.DEFAULT_AIR_DENSITY;
        }

        // Determine Airspeed: Priority Sensor -> Fallback Ground Speed
        var sAirSpeed = (Utilities.USING_SENSOR && sParams != null && sParams.size() > Utilities.AIR_SPEED_INX) ? sParams[Utilities.AIR_SPEED_INX] : null;
        if (sAirSpeed != null && (sAirSpeed instanceof Lang.Number || sAirSpeed instanceof Lang.Float)) {
            var sensorValue = sAirSpeed.toFloat();
            if (Utilities.TESTING_WITH_FAN) {
                // Virtual Headwind: When testing with a fan and FIT playback,
                // the total airspeed is the sum of virtual ground speed and fan speed.
                speedAirSensorMSec = rawSpeed + sensorValue;
            } else {
                // Standard operation: Sensor measures total relative airspeed directly
                speedAirSensorMSec = sensorValue;
            }
        } else {
            speedAirSensorMSec = rawSpeed; // Default to Ground Speed
        }

        // Guard: Only exit if not fan testing AND speed is low (NaN safe check)
        if (!Utilities.TESTING_WITH_FAN && (rawSpeed == null || rawSpeed < 0.5f)) { 
            if (Utilities.DEBUG) { printDebugTable(rawAlt); }
            return (CdAValue > 0) ? CdAValue : 0.0f; 
        }

        var isTimerRunning = (info.timerState != null && info.timerState == Activity.TIMER_STATE_ON);

        if (!_hasValidAltitude && hasGarminAlt) { // Initial state, no previous altitude data
            speedMSecPrevious  = rawSpeed;
            altitudePrevious   = rawAlt;
            _garminAltPrevious = rawAlt;
            altitude           = rawAlt;
            _hasValidAltitude  = true;
        } else {
            // Garmin altitude is already filtered by the head unit. Applying a
            // second, slow Kalman filter in internal-only mode makes vertical
            // speed lag badly; after a descent it can still report a negative
            // climb value (#) after the rider has started climbing.
            if (hasGarminAlt && sParams != null && _altKalman != null) {
                altitude = (_altKalman as KalmanFilter).update(rawAlt);
            } else if (hasGarminAlt) {
                altitude = rawAlt;
            }
        }

        if (samplesCollected < duration) { samplesCollected++; }

        // 3. Update History Buffers and Rolling Sums

        currentSpeedSqDiff = (speedMSec * speedMSec) - (speedMSecPrevious * speedMSecPrevious);
        
        if (currentSpeedSqDiff != currentSpeedSqDiff || abs(currentSpeedSqDiff) < Utilities.KE_JITTER_THRESHOLD) { 
            currentSpeedSqDiff = 0.0f; 
        }

        var hPwr  = history_power;
        var hMSec = history_speedMSec;
        var hAir  = history_speedAirSensorMSec;
        var hDrag = history_dragPowerFactor;
        var hSqD  = history_speedSqDiff;
        var hGAltDiff = history_garmin_altitude_diff;

        // Update rolling sums and history buffers regardless of timer state for live display
        if (hPwr != null && hMSec != null && hAir != null && hDrag != null && hSqD != null && hGAltDiff != null) {
            // Safely subtract previous values, handling potential nulls if history was not fully populated
            if (hPwr[durationIdxNext]      != null) { sumOfPower -= hPwr[durationIdxNext] as Float; }
            if (hMSec[durationIdxNext]     != null) { sumOfSpeedMSec -= hMSec[durationIdxNext] as Float; }
            if (hAir[durationIdxNext]      != null) { sumOfSpeedAirSensorMSec -= hAir[durationIdxNext] as Float; }
            if (hDrag[durationIdxNext]     != null) { sumOfDragPowerFactor -= hDrag[durationIdxNext] as Float; }
            if (hSqD[durationIdxNext]      != null) { sumOfSpeedSqDiff -= hSqD[durationIdxNext] as Float; }
            if (hGAltDiff[durationIdxNext] != null) { sumOfFusedAltitudeDifferences -= hGAltDiff[durationIdxNext] as Float; }

            // Sanitize all values for NaN before they enter the rolling window to prevent sum pollution
            hPwr[durationIdxNext]  = (power == power) ? power : 0.0f;
            hMSec[durationIdxNext] = speedMSec;
            hAir[durationIdxNext]  = (speedAirSensorMSec == speedAirSensorMSec) ? speedAirSensorMSec : 0.0f;
            
            // PHYSICS FIX: Power = DragForce * Velocity_Road. 
            // For stationary fan testing, we use Airspeed as a surrogate for Ground Speed 
            // to allow the power-balance equation to function without actual forward motion.
            var vRoadFactor = (Utilities.TESTING_WITH_FAN && speedMSec < 0.1f) ? speedAirSensorMSec : speedMSec;
            var rawDrag = (speedAirSensorMSec * speedAirSensorMSec) * vRoadFactor;
            hDrag[durationIdxNext] = (rawDrag == rawDrag) ? rawDrag : 0.0f;
            hSqD[durationIdxNext]  = (currentSpeedSqDiff == currentSpeedSqDiff) ? currentSpeedSqDiff : 0.0f;
            
            // Sanitize altitude delta to prevent NaN propagation or GPS glitches
            var instantAltDelta = altitude - _garminAltPrevious;
            if (instantAltDelta != instantAltDelta || abs(instantAltDelta) > Utilities.IMPOSSIBLE_CLIMB_RATE) { 
                instantAltDelta = 0.0f;
            }
            // The history buffer below already averages this value across the
            // selected duration. Do not smooth Garmin-only altitude a second
            // time, because the extra EMA preserves the previous slope and can
            // display downhill power while the rider is climbing.
            garminAltDelta = (sParams != null)
                ? lowPassFilter(instantAltDelta, garminAltDelta)
                : instantAltDelta;

            // 1-second Fusion Logic: Fuse instantaneous Garmin delta with instantaneous Sensor delta
            var sensorDelta = (sParams != null && sParams.size() > Utilities.ALT_DIFF_INX) ? sParams[Utilities.ALT_DIFF_INX] : null;
            // getFilteredAltDelta may return an array of nullable Floats if inputs are null
            var altResult = $.getFilteredAltDelta(
                altitudePrevious, 
                sensorDelta, 
                altitude,
                garminAltDelta,
                null
            ) as Array<Float?>;
            
            // Sanitize fusedDelta: Use 0.0f if the fusion logic returns null to prevent crash in sum addition
            var fusedDeltaRaw = altResult[0];
            var fusedDelta = (fusedDeltaRaw != null && fusedDeltaRaw == fusedDeltaRaw) ? fusedDeltaRaw : 0.0f;
            altitudePrevious = altResult[1];

            hGAltDiff[durationIdxNext] = fusedDelta;
            
            sumOfPower              += power;
            sumOfSpeedMSec          += speedMSec;
            sumOfSpeedAirSensorMSec += speedAirSensorMSec;
            sumOfDragPowerFactor    += hDrag[durationIdxNext]; // hDrag[durationIdxNext] is assigned a Float above, so it's safe
            sumOfSpeedSqDiff        += currentSpeedSqDiff;
            sumOfFusedAltitudeDifferences += fusedDelta;

            // Advance the index only when we actually update the sums
            if (hMSec != null) { 
                durationIdxNext = (durationIdxNext + 1) % hMSec.size(); 
            }
        }

        // 4. Calculate Averages and Power Usage Components
        var divisor = (samplesCollected > 0) ? samplesCollected.toFloat() : 1.0f;
        powerAvg                = sumOfPower / divisor;
        speedAvgMSec            = sumOfSpeedMSec / divisor;
        speedAvgAirSensorMSec   = sumOfSpeedAirSensorMSec / divisor;
        speedAvgSqDiff          = sumOfSpeedSqDiff / divisor;
        avgDragFactor           = sumOfDragPowerFactor / divisor;
        avgVerticalSpeedMS      = sumOfFusedAltitudeDifferences / divisor; // Average vertical rate (m/s) over the duration

        if (Utilities.DEBUG) { printDebugTable(rawAlt); }

        // Apply physics with correct weight types: static mass for gravity/rolling, effective mass for inertia
        var rawClimb = (speedAvgMSec > 0.5f) ? (staticWeight * Utilities.ACCELERATION_GRAVITY * avgVerticalSpeedMS) : 0.0f;
        climbPowerUsage = (rawClimb == rawClimb) ? rawClimb : 0.0f; // NaN guard

        frictionPowerUsage = crrValue * staticWeight * Utilities.ACCELERATION_GRAVITY * speedAvgMSec;
        
        var rawKinetic = (effectiveMass * 0.5f * speedAvgSqDiff);
        kineticEnergyVariationPowerUsage = (rawKinetic == rawKinetic) ? rawKinetic : 0.0f; // NaN guard

        driveTrainPowerUsage = Utilities.DRIVE_TRAIN_EFFICIENCY * powerAvg;
        
        netPowerNoCdA = climbPowerUsage + frictionPowerUsage + kineticEnergyVariationPowerUsage + driveTrainPowerUsage;
        aeroPower     = powerAvg - netPowerNoCdA;

        // 5. Final CdA Calculation and State Update
        // Use rolling average of altitude change for gradient check to prevent UI flickering
        var currentGradient = (speedAvgMSec > 1.0f) ? (avgVerticalSpeedMS / speedAvgMSec) : 0.0f;
        // Safety: ensure currentGradient is not NaN and within valid aerodynamic testing range (+/- 2%)
        var isGradientValid = (currentGradient == currentGradient && (currentGradient >= -0.02f && currentGradient <= 0.02f));
        var currentDensity = airDensity;

        // Relax speed requirement for Fan Testing. The power balance equation still requires 
        // a non-zero ground speed (v_road) to calculate CdA from power.
        var speedThreshold = (Utilities.TESTING_WITH_FAN) ? 0.5f : 3.0f;
        var rawCheckSpeed = (Utilities.TESTING_WITH_FAN) ? speedAvgAirSensorMSec : speedAvgMSec;

        // Final check to prevent division by zero or NaN propagation
        var checkSpeed = (rawCheckSpeed == rawCheckSpeed) ? rawCheckSpeed : 0.0f;
        var checkAeroPower = (aeroPower == aeroPower) ? aeroPower : 0.0f;

        // Ensure avgDragFactor is not zero to avoid "Internal error" (Division by zero)
        var isDragValid = (avgDragFactor > 0.001f); // avgDragFactor is always a Float, no need for null check

        // Accept either numeric representation Connect IQ gives us here.
        var validDensity = (currentDensity != null && (currentDensity instanceof Lang.Number || currentDensity instanceof Lang.Float) && currentDensity > 0.1f);

        if (samplesCollected < duration) {
            waitReason = "Warm";
        } else if (!validDensity) {
            waitReason = "Rho";
        } else if (checkSpeed <= speedThreshold) {
            waitReason = "Slow";
        } else if (checkAeroPower <= 5.0f) {
            waitReason = "Power";
        } else if (!isDragValid) {
            waitReason = "Drag";
        } else if (!isGradientValid) {
            waitReason = "Slope";
        } else {
            waitReason = "Calc";
        }

	        if (samplesCollected >= duration && validDensity && 
	            checkSpeed > speedThreshold && checkAeroPower > 5.0f && isDragValid && isGradientValid) {

	            // Division Guard: prevent divide-by-zero or near-zero crashes
            var denominator = 0.5f * (currentDensity as Float) * avgDragFactor;
            var rawCdA = (denominator > 0.0001f) ? (checkAeroPower / denominator).toFloat() : -2.0f;
            // Keep invalid sentinels in the UI only; never write them to the FIT field.
            var isValidCdA = rawCdA == rawCdA && rawCdA > 0.0f && rawCdA < 2.0f;

            if (isValidCdA) {
                CdAValue = rawCdA;
                waitReason = "OK";
                if (_cdaField != null && isTimerRunning) {
                    (_cdaField as FitContributor.Field).setData(rawCdA);
                }
            } else {
                if (rawCdA == rawCdA && rawCdA >= 2.0f) {
                    waitReason = "CdA>2";
                } else if (rawCdA == rawCdA && rawCdA <= 0.0f) {
                    waitReason = "CdA<0";
                } else {
                    waitReason = "CdA";
                }
                CdAValue = -2.0f;
            }
        } else {
            CdAValue = (checkSpeed <= speedThreshold) ? -1.0f : -2.0f;
        }

        speedMSecPrevious = speedMSec;
        _garminAltPrevious = altitude;

        return (CdAValue > 0) ? CdAValue : 0.0f;
    }

    function onLayout(dc as Graphics.Dc) as Void {

        //! choose the correct layout before doing any measurements
        setLayout($.Rez.Layouts.MainLayout(dc));
        // common sizing logic stays the same
        var Font = new [4];

		Font[0] = Graphics.FONT_NUMBER_MILD;
		Font[1] = Graphics.FONT_NUMBER_MEDIUM;		
		Font[2] = Graphics.FONT_NUMBER_HOT;
		Font[3] = Graphics.FONT_NUMBER_THAI_HOT;

		var CdA_Pattern = "0.000";
        var targetHeight  = (dc.getHeight() / 4) - Graphics.getFontHeight(DF_Title_font) - 2;

   		for (var i = Font.size() - 1; i >= 0 ; --i)
   		{
   			CdA_Value_font = Font[i];
			if (
				(Graphics.getFontHeight(CdA_Value_font) <= targetHeight)
				&&
				(dc.getTextWidthInPixels(CdA_Pattern, CdA_Value_font) <= dc.getWidth() - 10)
			   )
			{
				break;
			}
   		}

		var Value_Pattern = "000/000/000"; // Increased pattern length to force a slightly smaller font size for the 3rd row
   		for (var i = Font.size() - 1; i >= 0 ; --i)
   		{
   			Value_font = Font[i];
			if (
				(Graphics.getFontHeight(Value_font) <= targetHeight)
				&&
				(dc.getTextWidthInPixels(Value_Pattern, Value_font) <= dc.getWidth() - 10)
			   )
			{
				break;
			}
   		}

		var Speed_Pattern = "00.0 / 00.0"; 
		for (var i = Font.size() - 1; i >= 0 ; --i)
		{
			Speed_Value_font = Font[i];
			if (
				(Graphics.getFontHeight(Speed_Value_font) <= targetHeight)
				&&
				(dc.getTextWidthInPixels(Speed_Pattern, Speed_Value_font) <= dc.getWidth() - 10)
			   )
			{	break;}
		}

		var Row_Pattern = "000 / 000"; // Sized for Climb / Kinetic
		for (var i = Font.size() - 1; i >= 0 ; --i)
   		{
   			Row_Value_font = Font[i];
			if (
				(Graphics.getFontHeight(Row_Value_font) <= targetHeight)
				&&
				(dc.getTextWidthInPixels(Row_Pattern, Row_Value_font) <= dc.getWidth() - 10)
			   )
			{
				break;
			}
   		}

		//Value_x = dc.getTextWidthInPixels(Value_Pattern, Value_font) ;
		Value_x = dc.getWidth() / 2;
		Value_y = DF_Title_y + Graphics.getFontHeight(DF_Title_font);
    }

    function onUpdate(dc as Graphics.Dc) as Void {
        // Garmin may suspend the normal DataField.compute() callback while the
        // activity timer is stopped. Keep the live calculation and display
        // updating, but leave FIT writes gated by timerState inside compute().
        var liveInfo = Activity.getActivityInfo();
        if (liveInfo.timerState == null || liveInfo.timerState != Activity.TIMER_STATE_ON) {
            compute(liveInfo);
        }

        var bgColor = getBackgroundColor();
        var FontDisplayColor = (bgColor == Graphics.COLOR_BLACK) ? Graphics.COLOR_WHITE : Graphics.COLOR_BLACK;
        
        // Fix: Update the background drawable with the current system theme color
        var background = View.findDrawableById("Background");
        if (background != null && background has :setColor) {
            // Ensure we handle the color as a specific type to avoid ambiguity 
            // on high-resolution Edge devices.
            if (bgColor instanceof Lang.Number) {
                background.setColor(bgColor as Lang.Number);
            }
        }

		WatchUi.DataField.onUpdate(dc);

		if (page == 0) {
            var rowHeight = dc.getHeight() / 4;
            
            // Use instantaneous speeds for the UI display to ensure responsiveness and 
            // visibility of sensor data (like wind/fans) while stationary.
            // Safe formatting for rendering (guard against NaN and Null)
            var curGSpd = (speedMSec != null && speedMSec == speedMSec) ? speedMSec * 3.6f : 0.0f;
            var curASpd = (speedAirSensorMSec != null && speedAirSensorMSec == speedAirSensorMSec) ? speedAirSensorMSec * 3.6f : 0.0f;
            var speedString = curGSpd.format("%.1f") + "/" + curASpd.format("%.1f");
            
            var pwrVal  = (powerAvg == powerAvg) ? powerAvg.format("%.0f") : "0";
            var dragVal = (aeroPower == aeroPower) ? aeroPower.format("%.0f") : "0";
            // Combine Rolling Resistance and Drivetrain loss for the "Friction" display
            var fricPower = frictionPowerUsage + driveTrainPowerUsage;
            var fricVal = (fricPower == fricPower) ? fricPower.format("%.0f") : "0";

            var isClimbNegative = (climbPowerUsage != null && climbPowerUsage == climbPowerUsage && climbPowerUsage < -0.000001);
            var isKineticNegative = (kineticEnergyVariationPowerUsage != null && kineticEnergyVariationPowerUsage == kineticEnergyVariationPowerUsage && kineticEnergyVariationPowerUsage < -0.000001);

            var climbVal = isClimbNegative ? "#" + (-climbPowerUsage).format("%.0f") : climbPowerUsage.format("%.0f");
            var kineticVal = isKineticNegative ? "#" + (-kineticEnergyVariationPowerUsage).format("%.0f") : kineticEnergyVariationPowerUsage.format("%.0f");
            var spacer = "/";

            // Row 1 Title: CdA (Segmented to color "NO SENSORS" red)
            if (sensorParams != null) {
                textJ(dc, Value_x, DF_Title_y, DF_Title_font, FontDisplayColor, Graphics.TEXT_JUSTIFY_CENTER, DF_Title_Text_1);
            } else {
                var s1 = DF_Title_Text_1;
                var s2 = " NO SENSORS ";
                
                var w1 = dc.getTextWidthInPixels(s1, DF_Title_font);
                var w2 = dc.getTextWidthInPixels(s2, DF_Title_font);
                var startX = Value_x - ((w1 + w2) / 2);
                dc.setColor(FontDisplayColor, Graphics.COLOR_TRANSPARENT);
                dc.drawText(startX, DF_Title_y, DF_Title_font, s1, Graphics.TEXT_JUSTIFY_LEFT);
                dc.setColor(Graphics.COLOR_RED, Graphics.COLOR_TRANSPARENT);
                dc.drawText(startX + w1, DF_Title_y, DF_Title_font, s2, Graphics.TEXT_JUSTIFY_LEFT);
            }

            var displayCdA = "---";
            if (CdAValue > 0) {
                displayCdA = CdAValue.format("%.3f");
            } else if (CdAValue == -1.0f) {
                displayCdA = "Slow";
            } else if (CdAValue == -2.0f) {
                if (samplesCollected < duration) {
                    displayCdA = "W:" + (duration - samplesCollected);
                } else {
                    displayCdA = waitReason;
                }
            }
            
            textJ(dc, Value_x, Value_y, CdA_Value_font, FontDisplayColor, Graphics.TEXT_JUSTIFY_CENTER, displayCdA);

            // Row 2: Speed (Ground / Air)
            textJ(dc, Value_x, DF_Title_y + rowHeight, DF_Title_font, FontDisplayColor, Graphics.TEXT_JUSTIFY_CENTER, DF_Title_Text_3);
            textJ(dc, Value_x, Value_y + rowHeight, Speed_Value_font, FontDisplayColor, Graphics.TEXT_JUSTIFY_CENTER, speedString);

            // Row 3: Power (W)
            textJ(dc, Value_x, DF_Title_y + (rowHeight * 2), DF_Title_font, FontDisplayColor, Graphics.TEXT_JUSTIFY_CENTER, DF_Title_Text_2);
            
            var wp1 = dc.getTextWidthInPixels(pwrVal, Value_font);
            var wp2 = dc.getTextWidthInPixels(dragVal, Value_font);
            var wp3 = dc.getTextWidthInPixels(fricVal, Value_font);
            var wps = dc.getTextWidthInPixels(spacer, Value_font);
            var totalRow3W = wp1 + wps + wp2 + wps + wp3;
            var startX3 = Value_x - (totalRow3W / 2);
            var yPos3 = Value_y + (rowHeight * 2);

            dc.setColor(FontDisplayColor, Graphics.COLOR_TRANSPARENT);
            dc.drawText(startX3, yPos3, Value_font, pwrVal, Graphics.TEXT_JUSTIFY_LEFT);
            dc.drawText(startX3 + wp1, yPos3, Value_font, spacer, Graphics.TEXT_JUSTIFY_LEFT);
            dc.drawText(startX3 + wp1 + wps, yPos3, Value_font, dragVal, Graphics.TEXT_JUSTIFY_LEFT);
            dc.drawText(startX3 + wp1 + wps + wp2, yPos3, Value_font, spacer, Graphics.TEXT_JUSTIFY_LEFT);
            dc.drawText(startX3 + wp1 + wps + wp2 + wps, yPos3, Value_font, fricVal, Graphics.TEXT_JUSTIFY_LEFT);

            // Row 4: Aero / Climb / Kinetic Breakdown
            textJ(dc, Value_x, DF_Title_y + (rowHeight * 3), DF_Title_font, FontDisplayColor, Graphics.TEXT_JUSTIFY_CENTER, DF_Title_Text_4);

            var wClimb = dc.getTextWidthInPixels(climbVal,   Row_Value_font);
            var ws = dc.getTextWidthInPixels(spacer,     Row_Value_font);
            var wKinetic = dc.getTextWidthInPixels(kineticVal, Row_Value_font);
            var totalRow4W = wClimb + ws + wKinetic;
            var startX4 = Value_x - (totalRow4W / 2);
            var yPos4 = Value_y + (rowHeight * 3);

            dc.setColor(isClimbNegative ? Graphics.COLOR_RED : FontDisplayColor, Graphics.COLOR_TRANSPARENT);
            dc.drawText(startX4, yPos4, Row_Value_font, climbVal, Graphics.TEXT_JUSTIFY_LEFT);
            dc.setColor(FontDisplayColor, Graphics.COLOR_TRANSPARENT);
            dc.drawText(startX4 + wClimb, yPos4, Row_Value_font, spacer, Graphics.TEXT_JUSTIFY_LEFT);
            dc.setColor(isKineticNegative ? Graphics.COLOR_RED : FontDisplayColor, Graphics.COLOR_TRANSPARENT);
            dc.drawText(startX4 + wClimb + ws, yPos4, Row_Value_font, kineticVal, Graphics.TEXT_JUSTIFY_LEFT);
        } 
    }

    private function printDebugTable(garminAltitude as Float) as Void {
        var sParams = sensorParams;
        var sensorAlt = (sParams != null && Utilities.ALT_INX < sParams.size()) ? sParams[Utilities.ALT_INX] : null;
        var sensorDelta = (sParams != null && Utilities.ALT_DIFF_INX < sParams.size()) ? sParams[Utilities.ALT_DIFF_INX] : null;

        var gAltText = garminAltitude.toString();
        var sAltText = (sensorAlt != null) ? sensorAlt.toString() : "N/A";
        var fAltText = altitude.toString();
        var gDeltaText = garminAltDelta.toString();
        var sDeltaText = (sensorDelta != null) ? sensorDelta.toString() : "N/A";
        var fDeltaText = avgVerticalSpeedMS.toString();

        // Report the delegate's real state rather than inferring connection
        // from sensor data, which cannot distinguish pairing from streaming.
        var bleStatus = Utilities.USING_SENSOR ? "NOT STARTED" : "OFF";
        if (_bleDelegate != null && _bleDelegate has :getConnectionStatus) {
            bleStatus = _bleDelegate.getConnectionStatus();
        }
        System.println("Alt G=" + gAltText + " S=" + sAltText + " F=" + fAltText
            + " | dAlt G=" + gDeltaText + " S=" + sDeltaText + " F=" + fDeltaText
            + " | BLE=" + bleStatus);
    }

	function textJ(dc as Graphics.Dc, x as Numeric, y as Numeric, font as Graphics.FontDefinition, color as Graphics.ColorValue, justify as Graphics.TextJustification, s as String?) as Void
	{
		if (s != null)
		{
			dc.setColor(color, Graphics.COLOR_TRANSPARENT);
			//dc.drawText(x, y, font, s, Graphics.TEXT_JUSTIFY_LEFT|Graphics.TEXT_JUSTIFY_VCENTER);
			dc.drawText(x, y, font, s, justify);
		}
	}

    // Custom version of Toybox.Math.abs to handle Numeric types
    private function abs(val as Numeric) as Numeric {
        if (val < 0) {
            return -val;
        }
        return val;
    }

	// Simple Exponential Moving Average (EMA) filter
	private function lowPassFilter(currentValue as Float, previousValue as Float) as Float {
		return (currentValue * _emaAlpha) + (previousValue * (1.0f - _emaAlpha));
	}

    /**
     * Centralized loading of user settings. Call from initialize() and 
     * from App.onSettingsChanged() via the view reference.
     */
    public function updateSettings() as Void {
        var oldDuration = duration;
        // 1. Duration (Averaging Window)
        try {
            var val = Application.Properties.getValue("AVG_Duration");
            if (val instanceof Lang.Number) {
                duration = (val < 1) ? 3 : val;
            } else if (val instanceof Lang.String) {
                var num = val.toNumber();
                duration = (num == null || num < 1) ? 3 : num;
            }
        } catch (e) {
            duration = 3;
        }
        _emaAlpha = 2.0f / (duration.toFloat() + 1.0f);

        // 2. Bike Weight (Kg)
        try {
            var val = Application.Properties.getValue("bikeWeight");
            if (val instanceof Lang.Float || val instanceof Lang.Number) {
                bikeWeight = val.toFloat();
            } else if (val instanceof Lang.String) {
                bikeWeight = val.toFloat();
            }
        } catch (e) {
            bikeWeight = 8.5f; 
        }

        // 3. Coefficient of Rolling Resistance (Crr)
        try {
            var val = Application.Properties.getValue("crrValue");
            if (val instanceof Lang.Float) {
                crrValue = val;
            } else if (val instanceof Lang.String) {
                crrValue = val.toFloat();
            }
        } catch (e) {
            crrValue = 0.0045f;
        }

        // 4. Use Sensor Setting
        try {
            var val = Application.Properties.getValue("use_sensor");
            if (val instanceof Lang.Boolean) {
                Utilities.USING_SENSOR = val;
            } 
        } catch (e) {
            // Maintain current state if property is missing or invalid
        }

        // 5. Update Physics Model Mass
        var profile    = UserProfile.getProfile();
        var userWeight = (profile != null) ? profile.weight : null;
		if (userWeight != null && userWeight > 0) {
            var riderWeight = userWeight.toFloat() / 1000.0f;
            var bikeWeightKg = bikeWeight.toFloat();
            staticWeight     = riderWeight + bikeWeightKg;
		} else { 
            staticWeight  = 90.0f;
		}
        effectiveMass = staticWeight * Utilities.INERTIA_FACTOR;

        // 6. Update UI Labels
   	    DF_Title_Text_1 = "CdA " + duration + "s";
		DF_Title_Text_2 = "Pwr             /    Drag      /    Friction (W)";
		DF_Title_Text_3 = "Ground Speed    /  Air Speed (KmHr)  ";
        DF_Title_Text_4 = "   Climb       /     Kinetic (W)";

        // If the averaging window changed, we must re-initialize the history buffers
        // to prevent array out-of-bounds or incorrect sum calculations.
        if (duration != oldDuration) {
            setupHistoryBuffers();
        }
    }

    /**
     * Initializes history buffers based on the current 'duration' setting.
     * WARNING: Calling this mid-ride will reset your rolling averages.
     */
    private function setupHistoryBuffers() as Void {
        history_speedMSec            = new [duration] as Array<Float>;
        history_speedAirSensorMSec   = new [duration] as Array<Float>;
		history_power                = new [duration] as Array<Float>;
		history_garmin_altitude_diff = new [duration] as Array<Float>;
     	history_speedSqDiff          = new [duration] as Array<Float>;
		history_dragPowerFactor      = new [duration] as Array<Float>;

        // Reset rolling sums to maintain consistency with new buffer sizes
        durationIdxNext    = 0;
        samplesCollected   = 0;
        sumOfPower         = 0.0f;
        sumOfSpeedMSec     = 0.0f;
        sumOfSpeedAirSensorMSec = 0.0f;
        sumOfDragPowerFactor    = 0.0f;
        sumOfSpeedSqDiff        = 0.0f;
        sumOfFusedAltitudeDifferences = 0.0f;

        var hMSec = history_speedMSec;
        var hAir  = history_speedAirSensorMSec;
        var hPwr  = history_power;
        var hGAltDiff = history_garmin_altitude_diff;
        var hSqD      = history_speedSqDiff;
        var hDrag     = history_dragPowerFactor;
       
        if (hMSec != null && hAir != null && hPwr != null && hGAltDiff != null && hSqD != null && hDrag != null) {
            for (var i = 0; i < duration; ++i) {
                hMSec[i] = 0.0f;
                hAir[i]  = 0.0f;
                hPwr[i]  = 0.0f;
                hSqD[i]  = 0.0f;
                hDrag[i] = 0.0f;
                hGAltDiff[i] = 0.0f;
            }
        }
    }

}
