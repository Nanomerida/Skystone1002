package org.firstinspires.ftc.teamcode.hardware;


import org.openftc.revextensions2.ExpansionHubEx;
import android.graphics.Color;

public class HubLED {


    private ExpansionHubEx hub;


    private int ledColor;

    public void setLedColor(int colorInt){
        hub.setLedColor(Color.red(colorInt), Color.green(colorInt), Color.blue(colorInt));
    }

    /** Retuns the current color int */
    public int getLedColor() {
        return ledColor;
    }
}
