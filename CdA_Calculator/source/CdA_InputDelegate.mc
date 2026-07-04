using Toybox.WatchUi;
using Toybox.System;
using Toybox.Application;
import Toybox.BluetoothLowEnergy;
import Toybox.Lang;
import Toybox.StringUtil;

class CdA_InputDelegate extends WatchUi.BehaviorDelegate {
    private var _view;

    function initialize(view as WatchUi.DataField) {
        BehaviorDelegate.initialize();
        _view = view;
    }

    function onKey(keyEvent as WatchUi.KeyEvent) as Boolean {
        var key = keyEvent.getKey();
        
        // Handle Up and Down buttons to toggle pages
        if (key == WatchUi.KEY_DOWN || key == WatchUi.KEY_UP) {
            // Use the global scope operator $. to resolve the custom class type
            if (_view instanceof $.CdA_CalculatorView) {
                var v = _view as $.CdA_CalculatorView;
                v.page = (v.page == 0) ? 1 : 0;
                WatchUi.requestUpdate();
            }
            return true;
        }
        return false;
    }
}