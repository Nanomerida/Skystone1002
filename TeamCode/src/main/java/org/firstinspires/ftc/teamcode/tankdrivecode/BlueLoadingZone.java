package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Variables.Reference;
import org.firstinspires.ftc.teamcode.CRVuforia.VuforiaBlue;


import static java.lang.Math.abs;
import static java.lang.Math.round;

@Autonomous (name = "BlueLoadingZone", group = "Autonomous")

public class BlueLoadingZone extends LinearOpMode {

    public DcMotor left_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor right_back_drive = null;
    //public Servo    claw   = null;
    //public DcMotor slide = null;
    //public DcMotor main_arm = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double ROBOT_WHEEL_DIST_INCHES = 8.5f;     // distance from center of robot to wheels
    static final double COUNTS_PER_WHEEL_REV = 96;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_MM = 90;     // For figuring circumference
    static final float COUNTS_PER_INCH = 2.9452f;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    /* Other Variables */
    public static final double degreesToRadians = 180.0 / Math.PI;
    private VuforiaBlue blockPosBlue = new VuforiaBlue();
    private Reference ref = new Reference();
    public static final double SERVODEGREES = 0.005;

    private void setRunToPosition() {
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetDrive() {
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void waitForDrive() {

        while (left_front_drive.isBusy() || left_back_drive.isBusy() || right_front_drive.isBusy() || right_back_drive.isBusy()) {
            sleep(1);
        }
    }

    private void moveDrive(float power, float inches) {
        resetDrive();

        left_front_drive.setPower(power);
        left_back_drive.setPower(power);
        right_front_drive.setPower(-power);
        right_back_drive.setPower(-power);

        left_front_drive.setTargetPosition( round(inches / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition( round(inches / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition( round(inches / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition( round(inches / COUNTS_PER_INCH));

        setRunToPosition();

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

        //set desired power
        left_front_drive.setPower(power * speedLeft);
        left_back_drive.setPower(power * speedLeft);
        right_front_drive.setPower(power * speedRight);
        right_back_drive.setPower(power * speedRight);

        //TURN
        left_front_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));

        setRunToPosition();

        waitForDrive();


    }


    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;
    //NOTE: to get heading, do degreesConversion()

    @Override
    public void runOpMode() {

        left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        //main_arm = hardwareMap.get(DcMotor.class, "main_arm");
        //slide = hardwareMap.get(DcMotor.class, "slide");




        waitForStart();

        moveDrive(1.0f, 26);
        waitForDrive();
        sleep(500);

        turnDrive("cw", 0.5f, 90);
        waitForDrive();
        sleep(500);

        moveDrive(0.7f, 25);
        waitForDrive();

        resetDrive();









    }


}


















