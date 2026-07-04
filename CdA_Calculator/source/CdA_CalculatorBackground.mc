import Toybox.Lang;
using Toybox.WatchUi as Ui;
using Toybox.Application as App;
using Toybox.Graphics as Gfx;


class Background extends Ui.Drawable
{

    private var mColor as Gfx.ColorValue = Gfx.COLOR_BLACK;

    function initialize()
    {
        var dictionary = {
            :identifier => "Background"
        };

        Drawable.initialize(dictionary);
    }

    function setColor(color as Gfx.ColorValue) as Void
    {
        mColor = color;
    }

    function draw(dc as Gfx.Dc) as Void
    {
        dc.setColor(Gfx.COLOR_TRANSPARENT, mColor);
        dc.clear();
    }

}
