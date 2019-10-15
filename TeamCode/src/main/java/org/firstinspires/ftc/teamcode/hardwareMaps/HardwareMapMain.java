package org.firstinspires.ftc.teamcode.hardwareMaps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 */
public class HardwareMapMain {
    /* Public OpMode members. */
    public DcMotor  left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;
    public DcMotor  main_arm     = null;
    public DcMotor  slide = null;
    public Servo    claw_level    = null;
    public Servo    claw   = null;
    public Servo    claw_rotate = null;

    public static final double START_POSITION_CLAW       =  0.0 ; //starting pose of main claw servo
    public static final double START_POSITION_CLAW_LEVELER = 0.0; //starting pose of the claw leveler
    public static final double START_POSITION_CLAW_ROTATER = 0.0;
    public static final double ARM_UP_POWER    =  0.45 ; //change this based on what we need
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMapMain(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        //Drive
        left_front_drive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hwMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hwMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hwMap.get(DcMotor.class, "rightBackDrive");

        //Arm
        slide = hwMap.get(DcMotor.class, "slide_motor");
        main_arm    = hwMap.get(DcMotor.class, "main_arm");
        claw_level = hwMap.get(Servo.class, "claw_leveler");
        claw = hwMap.get(Servo.class, "claw");
        claw_rotate = hwMap.get(Servo.class, "claw_rotate");


        // Set all motors to zero power
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        slide.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        main_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        main_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); //I don't know what we want for this

        // Define and initialize ALL installed servos.
        claw_level.setPosition(START_POSITION_CLAW_LEVELER);
        claw.setPosition(START_POSITION_CLAW);
        claw_rotate.setPosition(START_POSITION_CLAW_ROTATER);
    }

    public void resetEncoders(){
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setRunToPosition(){
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRunWithEncoders(){
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopDrive(){
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);
    }
    public void stopArm(){
        main_arm.setPower(0);
    }
    public void stopSlide(){
        slide.setPower(0);
    }
}
