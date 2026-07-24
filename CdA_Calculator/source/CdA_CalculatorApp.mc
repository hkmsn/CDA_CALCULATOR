
using  Toybox.Application as App;
import Toybox.Application.Storage;
import Toybox.Lang;
using  Toybox.BluetoothLowEnergy as Ble;
using  Toybox.System;
import Toybox.WatchUi;
import Utilities;


// now bluetooth sensor in Garmin Simulator /dev/cu.usbmodemD675FEE857722
// Current USB Port: /dev/cu.usbmodemXXXX (Update this after running 'ls /dev/cu.usbmodem*')
// Sim on Mac, fails at a low level, when using BLE with dongle, so can't be caught
// so, always use USING_SENSOR = false, when no sensor attached.
// TESTING_WITH_FAN adds airspeed to groundspeed from Sim FIT file run.

class CdA_CalculatorApp extends App.AppBase{

private var _bleDelegate as CdA_CalculatorBleDelegate?;
private var _view as CdA_CalculatorView?;
var sensorValue as Float?;

    function onStart(state as Dictionary?) as Void {
        refreshSensorSetting();
        System.println("CdA: app started [alt-debug-2]");
        if (Utilities.BLE_DEBUG) {
            System.println("BLE: App start; external sensor enabled=" + Utilities.USING_SENSOR);
        }
        if (Utilities.USING_SENSOR) {
            var delegate = ensureBleDelegate();
            if (delegate != null) {
                delegate.startScanning();
            }
        }
    }

    function initialize()
    {
        AppBase.initialize();
        refreshSensorSetting();
    }

    //! Safely load the 'use_sensor' preference
    private function refreshSensorSetting() as Void {
        try {
            var val = Application.Properties.getValue("use_sensor");
            if (val instanceof Lang.Boolean) {
                Utilities.USING_SENSOR = val;
            }
        } catch (e) {
            Utilities.USING_SENSOR = false;
        }
    }

    //! Ensures the BLE delegate is instantiated and registered with the stack
    private function ensureBleDelegate() as CdA_CalculatorBleDelegate? {
        if (Utilities.USING_SENSOR) {
            if (_bleDelegate == null) {
                _bleDelegate = new CdA_CalculatorBleDelegate();
                Ble.setDelegate(_bleDelegate);
            }
        }
        return _bleDelegate;
    }

    //! This is called by the system when settings are updated via the Simulator or Garmin Connect Mobile
    function onSettingsChanged() as Void {
        var wasUsingSensor = Utilities.USING_SENSOR;
        refreshSensorSetting();
        
        // Handle runtime toggle of BLE sensor
        if (Utilities.USING_SENSOR) {
            var delegate = ensureBleDelegate();
            if (delegate != null) {
                setupBleLinkage();
                delegate.updateAppSensorSettings(); // Update delegate's internal flags
                if (!wasUsingSensor) {
                    delegate.startScanning();
                }
            }
        } else if (wasUsingSensor) {
            if (_bleDelegate != null) {
                _bleDelegate.updateAppSensorSettings();
                _bleDelegate.stopScanning();
            }
        }

        // Safely trigger the view to reload all user settings
        if (_view != null && _view has :updateSettings) {
            (_view as CdA_CalculatorView).updateSettings();
        }

        WatchUi.requestUpdate();
        System.println("Settings updated and reloaded.");
    }

    //! onStart() is called on application start up

    function getInitialView() as [ WatchUi.Views ] or [ WatchUi.Views, WatchUi.InputDelegates ] {
    // 1. Get the setting (ensure this function exists and returns a value)
    var duration = readAppPropertyInt("AVG_Duration", 5); // Use readAppPropertyInt for user settings
    // 2. Create the View (Pass the single integer)
    // The View's initialize method expects an array of parameters.
    _view = new CdA_CalculatorView([duration] as Array<Lang.Any>?);
    
    setupBleLinkage();

    return [ _view as CdA_CalculatorView, new CdA_InputDelegate(_view as CdA_CalculatorView) ] as [ WatchUi.Views, WatchUi.InputDelegates ];
}

    //! Establish the connection between the BLE Delegate and the UI View
    private function setupBleLinkage() as Void {
        var delegate = _bleDelegate;
        var view = _view;
        if (delegate != null && view != null) {
            delegate.setView(view);
            delegate.updateAppSensorSettings(); // Ensure delegate's internal flags are updated
            view.setBleDelegate(delegate);
        }
    }

    function onStop(state as Dictionary?) as Void
    {
        // It's good practice to clean up BLE resources to prevent issues.
        if (_bleDelegate != null) {
            _bleDelegate.stopScanning();
        }
    }

	// Renamed to reflect reading from Application.Properties
	function readAppPropertyInt(key as App.PropertyKeyType, thisDefault as Number) as Number
	{
		// User settings are stored in Application.Properties, not Storage
		var value = Application.Properties.getValue(key);
        if (value instanceof Number) {
            return value;
        }
        // Safely attempt to convert string values to numbers
        if (value instanceof Lang.String) {
            try {
                return (value as Lang.String).toNumber();
            } catch (e) {
                System.println("Warning: Property '" + key + "' has non-numeric string value: " + value + ". Using default.");
            }
        }
        return thisDefault;
	}
}
