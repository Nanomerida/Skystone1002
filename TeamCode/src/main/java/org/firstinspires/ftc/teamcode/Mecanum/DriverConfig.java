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
    public enum Claw controls {
        TRIGGERS,
        BUMPERS
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
        public LiftMode liftMode;
        public DirectionalControls liftControls, armControls;
        public Button liftSlowModeButton;
        public ClawControls clawControls;

        public Manipulator(List<Element> manipulatorList){
            liftMode = (manipulatorList.get(0).getText().equals("smooth")) ? LiftMode.SMOOTH : LiftMode.STAGED;
            switch (manipulatorList.get(1).getText()){
                 case "dpad": liftControls = DirectionalControls.DPAD;
                 case "left_stick": liftControls = DirectionalControls.LEFT_STICK;
                 case "right_stick": liftControls = DirectionalControls.RIGHT_STICK;
            }
            switch(manipulatorList.get(2).getText()){
                case "left_bumper": liftSlowModeButton = Button.LEFT_BUMPER;
                case "right_bumper": liftSlowModeButton = Button.RIGHT_BUMPER;
                case "x": liftSlowModeButton = Button.X;
                case "y": liftSlowModeButton = Button.Y;
                case "a": liftSlowModeButton = Button.A;
                case "b": liftSlowModeButton = Button.B;
                case "left_stick_button": liftSlowModeButton = Button.LEFT_STICK_BUTTON;
                case "right_stick_button": liftSlowModeButton = Button.RIGHT_STICK_BUTTON;
            }
            switch (manipulatorList.get(3).getText()){
                 case "dpad": armControls = DirectionalControls.DPAD;
                 case "left_stick": armControls = DirectionalControls.LEFT_STICK;
                 case "right_stick": armControls = DirectionalControls.RIGHT_STICK;
            }
             switch (manipulatorList.get(4).getText()){
                 case "triggers": armControls = ClawControls.TRIGGERS;
                 case "bumpers": armControls = ClawControls.BUMPERS;
            } 
            
            
        }
    }
}
