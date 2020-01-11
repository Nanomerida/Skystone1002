package org.firstinspires.ftc.teamcode.Mecanum.Config;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;


/** Contains the current configurations of the drive team. */
@TeleOp(name = "Driver Configuration", group = "Config")
//@Disabled
public class DriverConfig  extends LinearOpMode {


    private DriverControls driverControls;
    private ManipulatorControls manipulatorControls;

    Telemetry.Item currentQuery;
    Telemetry.Item currentDriver;

    public static String driverFileName = "DriverControls.json";
    public static String manipulatorFileName = "ManipulatorControls.json";



    Func<String> message = new Func<String>() {
        @Override
        public String value(){
            return queries[stepNumber];
        }
    };

    public enum DriverName {
        PARKER,
        JONAH
    }

    /**Enum to represent driver buttons and also the method to use them.
     *
     */
    public enum Button {
        A,
        B,
        X,
        Y,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON;

        public boolean isPressed(Gamepad gamepad){
            switch (this){
                case A: return gamepad.a;
                case B: return gamepad.b;
                case X: return gamepad.x;
                case Y: return gamepad.y;
                case LEFT_BUMPER: return gamepad.left_bumper;
                case RIGHT_BUMPER: return gamepad.right_bumper;
                case DPAD_UP: return gamepad.dpad_up;
                case DPAD_DOWN: return gamepad.dpad_down;
                case DPAD_LEFT: return gamepad.dpad_left;
                case DPAD_RIGHT: return gamepad.dpad_right;
                case LEFT_STICK_BUTTON: return gamepad.left_stick_button;
                case RIGHT_STICK_BUTTON: return gamepad.right_stick_button;
                default: return false;
            }
        }
    }

    public enum Trigger {
        LEFT_STICK_Y,
        LEFT_STICK_X,
        RIGHT_STICK_Y,
        RIGHT_STICK_X,
        LEFT_TRIGGER,
        RIGHT_TRIGGER;

        public float getValue(Gamepad gamepad){
            switch (this){
                case LEFT_STICK_X: return gamepad.left_stick_x;
                case LEFT_STICK_Y: return gamepad.left_stick_y;
                case RIGHT_STICK_X: return gamepad.right_stick_x;
                case RIGHT_STICK_Y: return gamepad.right_stick_y;
                case LEFT_TRIGGER: return gamepad.left_trigger;
                case RIGHT_TRIGGER: return gamepad.right_trigger;
                default: return 0;
            }
        }
    }


    private DriverName driverName = DriverName.PARKER;
    private int stepNumber = 0;


    @Override public void runOpMode()throws InterruptedException {

         telemetry.log().add("Welcome to Driver Config!");
         telemetry.log().add("Please refer to the instructions below,");
         telemetry.log().add("once start is pressed, this will begin.");
         telemetry.log().add(" ");
         telemetry.log().add("Driver will go first. When a control is ");
         telemetry.log().add("shown, please press the corresponding button.");
         telemetry.log().add("You might have to wait a second between each.");
         telemetry.log().add("AND PLEASE FOR THE LOVE OF GOD DO NOT PRESS MULTIPLE BUTTONS!!");
         telemetry.log().add("JUST BE PATIENT!!!");
         telemetry.log().add("Also: Please press buttons/move triggers until you see \"release\"");
         telemetry.log().add("And obviously, don't press a button twice!");


         telemetry.log().add("Waiting for start...");

         // Wait until we're told to go
         while (!isStarted()) {
             telemetry.update();
             idle();
         }

         telemetry.log().add("...started...");

         currentDriver = telemetry.addData("Current Driver", () -> driverName.toString());

         currentQuery = telemetry.addData("Current Control To be set", message);



        driverControls = new DriverControls();
        manipulatorControls = new ManipulatorControls();



        File driverFile = AppUtil.getInstance().getSettingsFile(driverFileName);
        File manipulatorFile = AppUtil.getInstance().getSettingsFile(manipulatorFileName);

        while(opModeIsActive()){

            if(stepNumber == 0 && buttonPressed1()){

                stepNumber++;

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

            }

            if(stepNumber == 1 && buttonPressed1()){

                registerButton(gamepad1, driverControls.slow_mode_button);

                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);

            }

            if(stepNumber == 2 && buttonPressed1()){
                registerButton(gamepad1, driverControls.reverse_mode_button);

                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
                driverName = DriverName.JONAH;
            }
            if(stepNumber == 3 && buttonPressed2()){
                registerButton(gamepad2, manipulatorControls.lift_up);


                currentQuery.setValue("Release");

                while(buttonPressed2()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 4 && buttonPressed2()){
                registerButton(gamepad2, manipulatorControls.lift_down);


                currentQuery.setValue("Release");

                while(buttonPressed2()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }

            if(stepNumber == 5 && buttonPressed2()){
                registerButton(gamepad2, manipulatorControls.claw_open);


                currentQuery.setValue("Release");

                while(buttonPressed2()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 6 && buttonPressed2()){
                registerButton(gamepad2, manipulatorControls.claw_closed);


                currentQuery.setValue("Release");

                while(buttonPressed2()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 7 && buttonPressed2()){
                registerButton(gamepad2, manipulatorControls.arm_down);


                currentQuery.setValue("Release");

                while(buttonPressed2()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 8 && buttonPressed2()){
                registerButton(gamepad2, manipulatorControls.arm_up);


                currentQuery.setValue("Release");

                while(buttonPressed2()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 9 && (gamepad2.left_stick_y != 0.0 || gamepad2.right_stick_y != 0.0)){
                if(gamepad2.left_stick_y != 0.0) manipulatorControls.foundation_movers = Trigger.LEFT_STICK_Y;
                else if(gamepad2.right_stick_y != 0.0) manipulatorControls.foundation_movers = Trigger.RIGHT_STICK_Y;


                currentQuery.setValue("Release");

                while(gamepad2.left_stick_y != 0.0 || gamepad2.right_stick_y != 0.0){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 10 && buttonPressed2()){
                registerButton(gamepad2, manipulatorControls.arm_up);

                if(gamepad2.a){

                    ReadWriteFile.writeFile(driverFile, serializeDriverControls());
                    ReadWriteFile.writeFile(manipulatorFile, serializeManipulatorControls());
                }



                stepNumber++;
            }



            telemetry.update();
        }






    }


    private String serializeDriverControls(){

            return SimpleGson.getInstance().toJson(driverControls);
    }

    private String serializeManipulatorControls(){

         return SimpleGson.getInstance().toJson(manipulatorControls);
    }

    private boolean buttonPressed1(){
         return (gamepad1.right_bumper || gamepad1.left_bumper || gamepad1.a || gamepad1.x || gamepad1.y ||
                 gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.left_stick_button
                || gamepad1.right_stick_button);
    }

    private boolean buttonPressed2(){
        return (gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.a || gamepad2.x || gamepad2.y ||
                gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.left_stick_button
                || gamepad2.right_stick_button);
    }

    private void registerButton(Gamepad gamepad, Button mapTo){
         if(gamepad.a) mapTo = Button.A;
         else if (gamepad.b) mapTo = Button.B;
         else if(gamepad.x) mapTo = Button.X;
         else if(gamepad.y) mapTo = Button.Y;
         else if(gamepad.left_bumper) mapTo = Button.LEFT_BUMPER;
         else if(gamepad.right_bumper) mapTo = Button.RIGHT_BUMPER;
         else if(gamepad.left_stick_button) mapTo = Button.LEFT_STICK_BUTTON;
         else if(gamepad.right_stick_button) mapTo = Button.RIGHT_STICK_BUTTON;
         else if(gamepad.dpad_up) mapTo = Button.DPAD_UP;
         else if(gamepad.dpad_down) mapTo = Button.DPAD_DOWN;
         else if(gamepad.dpad_left) mapTo = Button.DPAD_LEFT;
         else if(gamepad.dpad_right) mapTo = Button.DPAD_RIGHT;
    }


    /**
     * List with all the messages that appear.
     */
    private static final String[] queries = new String[]{
            "We will now do the Driver Configurations, press any button to continue.",
            "Now, press the desired button for the slow-mode toggle",
            "Button for reverse toggle",
            "Switch to Manipulator Gamepad. Button for Lift Up",
            "Button for Lift Down",
            "Button for Claw Open",
            "Button for Claw Closed",
            "Button for Arm Down",
            "Button for Arm Up",
            "Stick for foundation movers. Must be a stick!",
            "Done! Please press \"a\" to continue and wait for it to save.",
            "File Saved!!"
    };


    /**
     * Deserialize the json file for driver
     */
    public static DriverControls deserializeDriver(){

        File driverFile = AppUtil.getInstance().getSettingsFile(driverFileName);
        String data = ReadWriteFile.readFile(driverFile);
        return SimpleGson.getInstance().fromJson(data, DriverControls.class);
    }

    /**
     * Deserialize the json file for manipulator
     */
    public static ManipulatorControls deserializeManip(){

        File manipulatorFile = AppUtil.getInstance().getSettingsFile(manipulatorFileName);
        String data = ReadWriteFile.readFile(manipulatorFile);
        return SimpleGson.getInstance().fromJson(data, ManipulatorControls.class);
    }
}
