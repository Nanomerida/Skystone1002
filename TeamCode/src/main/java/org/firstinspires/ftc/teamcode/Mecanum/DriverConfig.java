package org.firstinspires.ftc.teamcode.Mecanum;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;


/** Contains the current configurations of the drive team. */
@Disabled
public class DriverConfig  extends LinearOpMode {


    private DriverControls driverControls;
    private ManipulatorControls manipulatorControls;

    Telemetry.Item currentQuery;
    Telemetry.Item currentDriver;


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


    private DriverName driverName = DriverName.PARKER;
    private int stepNumber = 0;

     class DriverControls  {

        public boolean slow_mode_button;
        public boolean reverse_mode_button;

    }

    /**Change this to all functions to get around the references!!!!
     *
     */
    class ManipulatorControls {

        public boolean lift_up;
        public boolean lift_down;
        public boolean claw_open;
        public boolean claw_closed;
        public boolean arm_down;
        public boolean arm_up;
        /**
         * Make sure to reverse this!
         */
        public float foundation_movers = gamepad2.right_stick_y;

    }

    @Override public void runOpMode(){

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


         telemetry.log().add("Waiting for start...");

         // Wait until we're told to go
         while (!isStarted()) {
             telemetry.update();
             idle();
         }

         telemetry.log().add("...started...");

         currentDriver = telemetry.addData("Current Driver", new Func<String>() {
             @Override
             public String value(){
                 return driverName.toString();
             }
         });

         currentQuery = telemetry.addData("Current Control To be set", message);



        driverControls = new DriverControls();
        manipulatorControls = new ManipulatorControls();

        String driverFileName = "DriverControls.json";
        String manipulatorFileName = "ManipulatorControls.json";

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
                if(gamepad2.left_stick_y != 0.0) manipulatorControls.foundation_movers = gamepad2.left_stick_y;
                else if(gamepad2.right_stick_y != 0.0) manipulatorControls.foundation_movers = gamepad2.right_stick_y;


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

    private void registerButton(Gamepad gamepad, boolean mapTo){
         if(gamepad.a) mapTo = gamepad.a;
         else if (gamepad.b) mapTo = gamepad.b;
         else if(gamepad.x) mapTo = gamepad.x;
         else if(gamepad.y) mapTo = gamepad.y;
         else if(gamepad.left_bumper) mapTo = gamepad.left_bumper;
         else if(gamepad.right_bumper) mapTo = gamepad.right_bumper;
         else if(gamepad.left_stick_button) mapTo = gamepad.left_stick_button;
         else if(gamepad.right_stick_button) mapTo = gamepad.right_stick_button;
    }


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
            "Trigger for foundation movers",
            "Done! Please press \"a\" to continue and wait for it to save.",
            "File Saved!!"
    };

    public static DriverControls deserializeDriver(String data){
        return SimpleGson.getInstance().fromJson(data, DriverControls.class);
    }

    public static ManipulatorControls deserializeManip(String data){
        return SimpleGson.getInstance().fromJson(data, ManipulatorControls.class);
    }
}
