package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Variables.Reference;
import org.firstinspires.ftc.teamcode.CRVuforia.VuforiaBlue;


import static java.lang.Math.abs;
import static java.lang.Math.round;

@Autonomous (name = "BlueLoadingZoneTank", group = "Autonomous")

public class BlueLoadingZone extends LinearOpMode {


    public DcMotor left_drive = null;
    public DcMotor right_drive = null;
    //public Servo    claw   = null;
    //public DcMotor slide = null;
    //public DcMotor main_arm = null;
    public WebcamName webcam = null;





    static final double ROBOT_WHEEL_DIST_INCHES = 14.3f;     // distance from center of robot to wheels
    static final double COUNTS_PER_WHEEL_REV = 288;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_MM = 88.9;     // For figuring circumference
    static final float COUNTS_PER_INCH = 2.9452f;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;





    /* Other Variables */
    public static final double degreesToRadians = 180.0 / Math.PI;
    private VuforiaBlue blockPosBlue = new VuforiaBlue();
    private ElapsedTime runtime = new ElapsedTime();
    private Reference ref = new Reference();
    public static final double SERVODEGREES = 0.005;




    //Methods
    private void setRunToPosition() {
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetDrive() {
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void waitForDrive() {
        while (left_drive.isBusy() || right_drive.isBusy()) {
            sleep(400);
        }
    }

    private void moveDrive(float power, float inches) {

        resetDrive();


        left_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        right_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));

        //FYI: might go after setPower
        setRunToPosition();

        left_drive.setPower(power);
        right_drive.setPower(power);




        waitForDrive();

    }

    public void turnDrive(String direction, double power, double degrees) {

        String cc = "cw";

        double inches = (degrees * degreesToRadians) * ROBOT_WHEEL_DIST_INCHES;
        float speedLeft = 1.0f;
        float speedRight = 1.0f;

        if(direction.equals(cc)){
            speedRight = -1.0f;
        }
        else{
            speedLeft = -1.0f;
        }

        resetDrive();

        left_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        right_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));

        //set desired power
        left_drive.setPower(power * speedLeft);
        right_drive.setPower(power * speedRight);

        setRunToPosition();

        waitForDrive();


    }


    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;
    //NOTE: to get heading, do degreesConversion()



    //Actual Code that runs during Auton
    @Override
    public void runOpMode() {

        left_drive  = hardwareMap.get(DcMotor.class, "leftDrive");
        right_drive = hardwareMap.get(DcMotor.class, "rightDrive");
        //main_arm = hardwareMap.get(DcMotor.class, "main_arm");
        //slide = hardwareMap.get(DcMotor.class, "slide");
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        blockPosBlue.blueInit(webcam);

//set target position, power, set run to position, wait for drive


        waitForStart();
        /**sleep(10000);
        //line up robot to left line of foam tile
        moveDrive(1.0f, 26);*/



        moveDrive(1f,3);

        /*
        switch (blockPosBlue) {
            case 0:
                turnDrive("ccw", 0.5f, 90);
                moveDrive(1f,3.5f);
                turnDrive("ccw",0.5f,90);
                //arm.setPower(1);
                //claw.setPosition(-0.5);
                break;
            case 1:
                break;
            case 2:
                break;
        }*/
        /**
        moveDrive(0.7f, 47);
        //claw.setPosition(0.5);

        moveDrive(-0.7f, 25);

        resetDrive();

        left_drive.setPower(0);
        right_drive.setPower(0);
        //arm.setPower(0);*/
    }


}


















