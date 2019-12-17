package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for the robot.
 * Just do robot.init(hardwareMap)
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left Front drive motor:       "leftFrontDrive""
 * Motor channel:  Left Back drive motor:        "leftBackDrive"
 * Motor channel:  Right Front drive Motor:      "rightFrontDrive"
 * Motor channel:  Right Back drive Motor:       "rightBackDrive"
 * Motor channel:  Arm main motor:               "main_arm"
 * Motor channel:  Linear Slide motor:           "slide_motor"
 * Servo channel:  Servo to control claw level:  "claw_leveler"
 * Servo channel:  Servo to open claw:           "claw"
 * Servo channel:  Servo to rotate claw:         "clawRotate"
 * Servo channel:  Servo to grab foundation:     "front_servo"
 */
public class HardwareMapMain {
    /* Public OpMode members. */
    public DcMotor  left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;
    public DcMotor  lift_motor = null;
    public CRServo intake_wheel_left = null;
    public CRServo intake_wheel_right = null;



    /* local OpMode members. */
    OpMode opMode;

    /* Constructor */
    public HardwareMapMain(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(OpMode aopMode) {
        // Save reference to Hardware map
        opMode = aopMode;

        // Define and Initialize Motors

        //Drive
        left_front_drive  = opMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = opMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = opMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = opMode.hardwareMap.get(DcMotor.class, "rightBackDrive");

        //Intake
        intake_wheel_left = opMode.hardwareMap.get(CRServo.class, "intake_wheel_left");
        intake_wheel_right = opMode.hardwareMap.get(CRServo.class, "intake_wheel_right");

        //Lift
        lift_motor = opMode.hardwareMap.get(DcMotor.class, "lift_motor");


        //Reset ALL encoders
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void stopDrive(){
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);
    }
}
