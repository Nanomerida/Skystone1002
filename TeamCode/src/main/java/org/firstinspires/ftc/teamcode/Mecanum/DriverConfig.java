package org.firstinspires.ftc.teamcode.Mecanum;


import org.jdom2.Element;

import java.util.List;

/** Contains the current configurations of the drive team. */

public class DriverConfig {

    public enum LiftMode {
        SMOOTH,
        STAGED
    }

    public enum DirectionalControls {
        DPAD,
        LEFT_STICK,
        RIGHT_STICK
    }

    public enum Button {
        RIGHT_BUMPER,
        LEFT_BUMPER,
        X,
        Y,
        A,
        B,
        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON,
        START_BUTTON,
        GUIDE_BUTTON
    }



    /** Configuration for driver */
    public static class Driver {
        public Button slowModeButton, fieldCentricButton;
        public boolean fieldCentric;

        public Driver(List<Element> driverPref){
            switch(driverPref.get(0).getText()){
                case "left_bumper": slowModeButton = Button.LEFT_BUMPER;
                case "right_bumper": slowModeButton = Button.RIGHT_BUMPER;
                case "x": slowModeButton = Button.X;
                case "y": slowModeButton = Button.Y;
                case "a": slowModeButton = Button.A;
                case "b": slowModeButton = Button.B;
                case "left_stick_button": slowModeButton = Button.LEFT_STICK_BUTTON;
                case "right_stick_button": slowModeButton = Button.RIGHT_STICK_BUTTON;
            }
            fieldCentric = driverPref.get(1).getText().equals("yes");
            switch(driverPref.get(2).getText()){
                case "left_bumper": slowModeButton = Button.LEFT_BUMPER;
                case "right_bumper": slowModeButton = Button.RIGHT_BUMPER;
                case "x": slowModeButton = Button.X;
                case "y": slowModeButton = Button.Y;
                case "a": slowModeButton = Button.A;
                case "b": slowModeButton = Button.B;
                case "left_stick_button": slowModeButton = Button.LEFT_STICK_BUTTON;
                case "right_stick_button": slowModeButton = Button.RIGHT_STICK_BUTTON;
            }

        }
    }

    /**Configuration for manipulator */
    public static class Manipulator {
        public boolean f;

        public Manipulator(List<Element> manipulatorList){

        }
    }
}
