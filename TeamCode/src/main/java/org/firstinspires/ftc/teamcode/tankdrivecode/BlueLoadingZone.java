package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Methods.GeneralMethods;
import org.firstinspires.ftc.teamcode.Variables.Reference;
import org.firstinspires.ftc.teamcode.VuforiaBlue;
import org.firstinspires.ftc.teamcode.VuforiaRed;
import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;


import static java.lang.Math.abs;
import static java.lang.Math.PI;
import static java.lang.Math.round;

@Autonomous (name = "BlueLoadingZone", group = "Autonomous")

public class BlueLoadingZone extends LinearOpMode {

    public DcMotor left_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor right_back_drive = null;
    //public Servo    claw   = null;
    public DcMotor slide = null;
    public DcMotor main_arm = null;

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
        right_front_drive.setPower(power);
        right_back_drive.setPower(power);

        left_front_drive.setTargetPosition( round(inches / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition( round(inches / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition( round(inches / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition( round(inches / COUNTS_PER_INCH));

    }

    public void turnDrive(String direction, double power, double degrees) {

        String cc = "cw";

        double inches = (degrees * degreesToRadians) * ROBOT_WHEEL_DIST_INCHES;

        if(direction.equals(cc)){
            right_front_drive.setDirection(DcMotor.Direction.REVERSE);
            right_back_drive.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            left_front_drive.setDirection(DcMotor.Direction.REVERSE);
            left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        }

        resetDrive();

        //set desired power
        left_front_drive.setPower(power);
        left_back_drive.setPower(power);
        right_front_drive.setPower(power);
        right_back_drive.setPower(power);

        //TURN
        left_front_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));

        setRunToPosition();

        waitForDrive();

        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_back_drive.setDirection(DcMotor.Direction.FORWARD);

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
        main_arm = hardwareMap.get(DcMotor.class, "main_arm");
        slide = hardwareMap.get(DcMotor.class, "slide");

        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        moveDrive(1.0f, );






    }


}

















}
