import Toybox.BluetoothLowEnergy;
import Toybox.Lang;
import Toybox.System;
import Toybox.WatchUi;
import Toybox.StringUtil;
import Utilities;

class CdA_CalculatorBleDelegate extends BluetoothLowEnergy.BleDelegate {
    
    // BLE UUID constants
    private const SERVICE_UUID        = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
    private const CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    private const TARGET_NAME         = "ESP32_Aero";

    private var _service    = BluetoothLowEnergy.stringToUuid(SERVICE_UUID);
    private var _char       = BluetoothLowEnergy.stringToUuid(CHARACTERISTIC_UUID);

    // Connection Hint: ESP32 setMinPreferred(0x06) [7.5ms] is used 
    // to minimize latency for the 4Hz sensor data stream.
    private var _dataBuffer = "";
    private var _view;
    private const MAX_BUFFER_SIZE     = 512;
    // With ESP32 at 4Hz, a window of 4 provides the best 1-second average for the 1Hz UI.
    private const ROLLING_WINDOW_BLE  = 4; 
    private var _rollingData  as Array<Array<Float?>?>; 
    private var _rollingIndex as Number = 0; 

    private var _device as BluetoothLowEnergy.Device? = null;
    private var _notificationsEnabledConfirmed as Boolean = false; 
    private var _updateSkipCount as Number = 0;
    private var _isBusy as Boolean = false;
    private var _connectionStatus as String = "OFF";
    private var _hasLoggedValidPacket as Boolean = false;
    private var _scanResultCount as Number = 0;
    private var _rxFragmentCount as Number = 0;

    // Snapshot variables for runtime performance
    private var _usingSensor as Boolean   = false;
    private var _expectedParams as Number = 7;
    private var _altInx as Number         = 5;
    private var _bleDebug as Boolean = false;
  
    function initialize() {
        BleDelegate.initialize();

        _usingSensor    = Utilities.USING_SENSOR;
        _expectedParams = Utilities.EXPECTED_SENSOR_PARAMS;
        _altInx         = Utilities.ALT_DIFF_INX;
        _bleDebug       = Utilities.BLE_DEBUG;

        BluetoothLowEnergy.registerProfile({
            :uuid => _service,
            :characteristics => [ { :uuid => _char, :descriptors => [ BluetoothLowEnergy.cccdUuid() ] } ]
        });
        BluetoothLowEnergy.setScanState(BluetoothLowEnergy.SCAN_STATE_SCANNING);
        _connectionStatus = "SCANNING";

        if (_bleDebug) {
            System.println("BLE Simple: Scanning for UUID " + _service.toString());
        }
        _rollingData = new [ROLLING_WINDOW_BLE];
    }

    public function registerBleProfile() as Void {
        // Profile registration is now handled in initialize()
    }

    public function startScanning() as Void {
        BluetoothLowEnergy.setScanState(BluetoothLowEnergy.SCAN_STATE_SCANNING);
        _connectionStatus = "SCANNING";
    }

    public function stopScanning() as Void {
        BluetoothLowEnergy.setScanState(BluetoothLowEnergy.SCAN_STATE_OFF);
        _connectionStatus = "OFF";
    }

    public function getConnectionStatus() as String {
        return _connectionStatus;
    }

    // Called by the App to update delegate's internal settings
    public function updateAppSensorSettings() as Void {
        _usingSensor    = Utilities.USING_SENSOR;
        _expectedParams = Utilities.EXPECTED_SENSOR_PARAMS; // Less likely to change, but good to keep consistent
        _altInx         = Utilities.ALT_DIFF_INX;         // Less likely to change, but good to keep consistent
        _bleDebug       = Utilities.BLE_DEBUG;
        if (!_usingSensor) {
            _connectionStatus = "OFF";
        }
    }

    function setView(view as WatchUi.DataField) as Void {
        _view = view;
        if (view has :setBleDelegate) {
            view.setBleDelegate(self);
        }
    }

    function onConnectedStateChanged(device as BluetoothLowEnergy.Device, state as BluetoothLowEnergy.ConnectionState) as Void {
        if (state == BluetoothLowEnergy.CONNECTION_STATE_CONNECTED) {
            if (_bleDebug) { System.println("BLE: Connected. Starting discovery..."); }
            _connectionStatus = "DISCOVERING";
            _device = device;
            _isBusy = false;
            _notificationsEnabledConfirmed = false;
            _enableNotifications(device);
        } else {
            if (_bleDebug) { System.println("BLE: Disconnected. Re-scanning..."); }
            _connectionStatus = "DISCONNECTED";
            _device = null;
            _notificationsEnabledConfirmed = false;
            _isBusy = false;
            _rollingData = new [ROLLING_WINDOW_BLE];
            // Clear data in the view so it shows "NO SENSORS" instead of "STALE DATA"
            if (_view != null) {
                var cdaView = _view as CdA_CalculatorView;
                cdaView.sensorParams = null;
            }
            startScanning();
        }
    }

    function onScanResults(scanResults as BluetoothLowEnergy.Iterator) as Void {
        if (_device != null) { return; } // Don't process scans if already connected
        for (var result = scanResults.next(); result != null; result = scanResults.next()) {
            if (result instanceof BluetoothLowEnergy.ScanResult) {
                var scanResult = result as BluetoothLowEnergy.ScanResult;
                var deviceName = scanResult.getDeviceName();
                var isMatch = deviceName != null && deviceName.equals(TARGET_NAME);
                _scanResultCount++;
                var uuids = scanResult.getServiceUuids();
                if (uuids != null) {
                    for (var uuid = uuids.next(); uuid != null; uuid = uuids.next()) {
                        if (uuid.equals(_service)) {
                            isMatch = true;
                            break;
                        }
                    }
                }

                if (_bleDebug) {
                    var shownName = (deviceName != null) ? deviceName : "<no name>";
                    System.println("BLE: Scan result " + _scanResultCount + "; name=" + shownName + "; match=" + isMatch);
                }

                // Some ESP32 advertising configurations provide the device name
                // but do not include the 128-bit service UUID in every packet.
                if (isMatch) {
                    if (_bleDebug) {
                        System.println("BLE: Found " + ((deviceName != null) ? deviceName : "sensor") + ". Pairing...");
                    }
                    BluetoothLowEnergy.setScanState(BluetoothLowEnergy.SCAN_STATE_OFF);
                    _connectionStatus = "PAIRING";
                    try {
                        BluetoothLowEnergy.pairDevice(scanResult);
                    } catch(e) {
                        if (_bleDebug) { System.println("Pairing Error: " + e.getErrorMessage()); }
                        _connectionStatus = "PAIR ERROR";
                        startScanning();
                    }
                    return;
                }
            }
        }
    }

    function attemptRetry() as Void {
        var dev = _device;
        if (_isBusy) {
            return; // Exit if an operation is already pending
        }

        if (dev != null && !_notificationsEnabledConfirmed) {
            _enableNotifications(dev);
        }
    }

    // Registered BLE services are available synchronously after connection.
    // Device.getServices() returns an iterator; it does not trigger an
    // onServices() callback.
    private function _enableNotifications(device as BluetoothLowEnergy.Device) as Boolean {
        var service = device.getService(_service);
        if (service == null) {
            _connectionStatus = "SERVICE MISSING";
            if (_bleDebug) { System.println("BLE: Registered service not found on device."); }
            return false;
        }

        var characteristic = service.getCharacteristic(_char);
        if (characteristic == null) {
            _connectionStatus = "CHAR MISSING";
            if (_bleDebug) { System.println("BLE: Sensor characteristic not found."); }
            return false;
        }

        var descriptor = characteristic.getDescriptor(BluetoothLowEnergy.cccdUuid());
        if (descriptor == null) {
            _connectionStatus = "CCCD MISSING";
            if (_bleDebug) { System.println("BLE: Notification descriptor not found."); }
            return false;
        }

        _connectionStatus = "ENABLING NOTIFY";
        _isBusy = true;
        if (_bleDebug) { System.println("BLE: Found descriptor. Enabling notifications..."); }
        descriptor.requestWrite([0x01, 0x00]b);
        return true;
    }

    function onProfileRegister(uuid as BluetoothLowEnergy.Uuid, status as BluetoothLowEnergy.Status) as Void {
        if (_bleDebug) { System.println("BLE: Profile registration status: " + status); }
        if (status != BluetoothLowEnergy.STATUS_SUCCESS) {
            _connectionStatus = "PROFILE ERROR";
        }
    }

    function onCharacteristicChanged(characteristic as BluetoothLowEnergy.Characteristic, value as Lang.ByteArray) as Void {
        if (!_usingSensor || value == null || value.size() == 0) { return; }
        _notificationsEnabledConfirmed = true;
        _connectionStatus = "RECEIVING";

        if (_dataBuffer.length() > MAX_BUFFER_SIZE) { _dataBuffer = ""; }

        var fragmentString = "";
        try {
            var valSize = value.size();
            var tmpArray = new [valSize] as Array<Number>;
            for (var i = 0; i < valSize; i++) { tmpArray[i] = value[i]; }
            fragmentString = StringUtil.utf8ArrayToString(tmpArray);
        } catch (e) { return; }

        if (fragmentString == null || fragmentString.length() == 0) { return; }
        _rxFragmentCount++;
        if (_bleDebug) {
            System.println("BLE: RX fragment " + _rxFragmentCount + " (" + value.size() + " bytes): " + fragmentString);
        }
        _dataBuffer += fragmentString;

        var terminatorPos = _dataBuffer.find("*");
        if (terminatorPos == null) { terminatorPos = _dataBuffer.find("\n"); }

        while (terminatorPos != null) {
            var message = _dataBuffer.substring(0, terminatorPos as Lang.Number);
            _dataBuffer = _dataBuffer.substring((terminatorPos as Lang.Number) + 1, null);
            if (message.length() > 0) { _processMessage(message); }
            terminatorPos = _dataBuffer.find("*");
            if (terminatorPos == null) { terminatorPos = _dataBuffer.find("\n"); }
        }
    }

    private function _processMessage(text as String) {
        var tokens = _split(text, "|");
        var numTokens = tokens.size();

        if (numTokens > 0 && tokens[numTokens - 1].length() == 0) {
            tokens = tokens.slice(0, numTokens - 1);
            numTokens = tokens.size();
        }

        // The live wire format is exactly EXPECTED_SENSOR_PARAMS fields:
        // Airspeed | Density | Altitude | AltChange
        if (numTokens != _expectedParams) {
            _connectionStatus = "RX FORMAT " + numTokens;
            if (_bleDebug) {
                System.println("BLE: Rejected packet with " + numTokens + " fields: " + text);
            }
            return;
        }

        var sensorParams = new [numTokens] as Array<Float?>;
        for (var i = 0; i < numTokens; i++) {
            var t = tokens[i];
            if (t != null && t.length() > 0 && !t.equals("NA")) {
                try {
                    sensorParams[i] = t.toFloat();
                } catch (e) {
                    _connectionStatus = "RX INVALID";
                    if (_bleDebug) { System.println("BLE: Invalid numeric field: " + t); }
                    return;
                }
            } else {
                sensorParams[i] = null;
            }
        }

        _connectionStatus = "STREAMING";
        if (_bleDebug && !_hasLoggedValidPacket) {
            System.println("BLE: First valid sensor packet received: " + text);
            _hasLoggedValidPacket = true;
        }

        _rollingData[_rollingIndex] = sensorParams;
        _rollingIndex = (_rollingIndex + 1) % ROLLING_WINDOW_BLE;

        if (_updateSkipCount == 0) {
            var averagedSensorParams = new [sensorParams.size()] as Array<Float?>;
            for (var i = 0; i < sensorParams.size(); i++) {
                var sum = 0.0f;
                var count = 0;
                var allNull = true;

                for (var j = 0; j < ROLLING_WINDOW_BLE; j++) {
                    var hist = _rollingData[j];
                    if (hist != null && hist.size() > i && hist[i] != null) {
                        sum += hist[i] as Float;
                        count++;
                        allNull = false;
                    }
                }

                if (allNull) {
                    averagedSensorParams[i] = null;
                } else {
                    averagedSensorParams[i] = (i == _altInx) ? sum : (sum / count);
                }
            }

            if (_view != null) {
                var cdaView = _view as CdA_CalculatorView;
                cdaView.sensorParams = averagedSensorParams;
                if (cdaView has :notifySensorUpdate) {
                    cdaView.notifySensorUpdate();
                }
            }
            _updateSkipCount = 3; 
        } else {
            _updateSkipCount--;
        }
    }

    private function _split(text as String, delimiter as String) as Array<String> {
        var tokens = [] as Array<String>;
        var remainder = text;
        var pos = remainder.find(delimiter);
        while (pos != null) {
            tokens.add(remainder.substring(0, pos));
            remainder = remainder.substring(pos + delimiter.length(), null);
            pos = remainder.find(delimiter);
        }
        tokens.add(remainder);
        return tokens;
    }

    function onDescriptorWrite(descriptor as BluetoothLowEnergy.Descriptor, status as BluetoothLowEnergy.Status) as Void {
        if (_bleDebug) { System.println("BLE: Descriptor Write Status: " + status); } 
        _isBusy = false;
        if (status == BluetoothLowEnergy.STATUS_SUCCESS || status == 15) {
            _notificationsEnabledConfirmed = true;
            _connectionStatus = "CONNECTED";
            if (_bleDebug) { System.println("BLE: Handshake Complete. Data streaming active."); }
        }
    }

    public function resetConnection() as Void {
        if (_bleDebug) { System.println("BLE: Hard Reset requested."); }        
        var dev = _device;
        _isBusy = false;
        _device = null; // Clear reference immediately to prevent race conditions
        _notificationsEnabledConfirmed = false;

        if (dev != null) {
            try {
                BluetoothLowEnergy.unpairDevice(dev);
            } catch (e) {
                if (_bleDebug) { System.println("BLE: Unpair failed: " + e.getErrorMessage()); }
            }
        }
        // In the Simulator, wait for the stack to breathe before restarting scan
        // The 15s watchdog in the View provides enough buffer for this.
        startScanning();
    }

    function onCharacteristicRead(c, s, v) {}
    function onCharacteristicWrite(c, s) {}
    function onDescriptorRead(d, s, v) {}
}
