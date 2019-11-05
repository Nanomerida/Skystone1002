package org.firstinspires.ftc.teamcode.Mecanum;


import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbTimeoutException;
import org.firstinspires.ftc.teamcode.Variables.RobotState;

/** A class to show telemetry in an organized manner on the DS screen.
 *
 * <p> This class contains many telemetry-orientated things to organize the Telemetry.
 * This contains the following Elements of the telemetry screen.</p>
 *
 * <br>
 *     <br>
 *  StartTelemetry.startDrive() Drive Base
 *  <br>
 *  StartTelemetry.startVision() Vuforia Vision
 *  <br>
 *  StartTelemetry.startStep() Current Step Number
 *  <br>
 *  StartTelemetry.startHeading() Current Heading (temperamental)
 *  <br>
 *  StartTelemetry.startHealth() Robot Health (temperamental)
 *  <br>
 *
 *  <br>
 *      <br>
 *
 * <p>The reason that these telemetry items have functional values is because opMode.telemetry.update
 * by default wipes the Telemetry screen. The exception to this are telemetry.addData() and telemetry.
 * Item things that contain functional values. Therefore these things have functions.</p>
 *
 *
 *
 * @author Ryan Driemeyer
 *
 *
 *
 *
 */

public class ShowTelemetry {

    private OpMode opMode;
    RobotState robotState;

    public ShowTelemetry(OpMode aOpMode, RobotState arobotState)
    {
        this.robotState = arobotState;
        this.opMode = aOpMode;

    } //constructor

    //value Strings
    private int stepNumbV = 0;
    private double currentHeadingV = 0;
    private String robotHealthV = "d";

    public class StartTelemetry {

        public StartTelemetry(){

        }

        /** Starts the drive base telemetry.
         *
         */
        public void startDrive(){
            opMode.telemetry.addData("Drive Base Status:", new Func<String>(){
                @Override public String value() {
                    return robotState.getDriveState().toString();
                }
            });
        }

        /** Starts the vision telemetry. */
        public void startVision() {
            opMode.telemetry.addData("Vision Testing Status:", new Func<String>(){
                @Override public String value(){
                    return robotState.getVisionState().toString();
                }
            });
        }

        /**Starts the step number telemetry. */
        public  void startStep(){
            opMode.telemetry.addData("Current Step Number:", new Func<Integer>(){
                @Override
                public Integer value() {
                    return stepNumbV;
                }
            });
        }


        /**Starts the heading telemetry */
        public void startMain(){
            opMode.telemetry.addData("Robot State:", new Func<String>(){
                @Override public String value() {
                    return robotState.getMainState().toString();
                }
            });
        }


        public void startIntake(){
            opMode.telemetry.addData("Intake Status:", new Func<String>(){
                @Override
                public String value() {
                    return robotState.getIntakeState().toString();
                }
            });
        }

        public void startLift(){
            opMode.telemetry.addData("Lift Status:", new Func<String>(){
                @Override
                public String value() {
                    return robotState.getLiftState().toString();
                }
            });
        }

    }





    ShowTelemetry.StartTelemetry startTelemetry = new ShowTelemetry.StartTelemetry();
    ShowTelemetry.UpdateTelem updateTelem = new ShowTelemetry.UpdateTelem();




    public void startTelemetry(){



        opMode.telemetry.addData("The Matrix:", "is ready!!");
        opMode.telemetry.update();
        startTelemetry.startDrive();
        startTelemetry.startIntake();
        startTelemetry.startLift();
        startTelemetry.startVision();
        startTelemetry.startStep();
        startTelemetry.startMain();
        opMode.telemetry.update();

    }


    public class UpdateTelem {
        public UpdateTelem() {

        }

        public void updateTelemetry() {
            opMode.telemetry.update();
        }


        public void setStepNumbV(int step) {
            stepNumbV = step;
        }

    }

    public void telemetryMessage(String caption, String value){opMode.telemetry.addData(caption, value);}

    public void telemetryMessage(String caption, double value) {opMode.telemetry.addData(caption, value);}








}
