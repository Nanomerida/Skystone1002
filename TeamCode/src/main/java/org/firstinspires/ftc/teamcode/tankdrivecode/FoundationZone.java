package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Variables.Reference;
import com.qualcomm.robotcore.hardware.CRServo;


import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.round;

@Autonomous (name = "FoundationZoneTank", group = "Autonomous")

public class FoundationZone extends LinearOpMode {
    public DcMotor left_drive;
    public DcMotor right_drive;
    public CRServo claw;
    public DcMotor arm;




//0.005259

    static final double ROBOT_WHEEL_DIST_INCHES = 5.5;     // distance from center of robot to wheels
    //static final double COUNTS_PER_WHEEL_REV = 288;
    static final double COUNTS_PER_WHEEL_REV = 2240;// eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * PI;
    //static final double COUNTS_PER_INCH = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE; // number of encoder ticks per inch circumference
    static final double COUNTS_PER_INCH =  ((WHEEL_CIRCUMFERENCE)/(COUNTS_PER_WHEEL_REV));





    /* Other Variables */
    public static final double degreesToRadians = 180.0 / Math.PI;
    //private VuforiaBlue blockPosBlue = new VuforiaBlue();
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
            idle();
        }
    }

    private void moveDrive(float power, float inches) {

        resetDrive();

        //left_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));
        //right_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));

        left_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        right_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));

        left_drive.setPower(power);
        right_drive.setPower(power);

        setRunToPosition();

        waitForDrive();

        left_drive.setPower(0);
        right_drive.setPower(0);
    }


    public void turnDrive(double tpower, double degrees) {
        //double inches = (degrees * degreesToRadians) * ROBOT_WHEEL_DIST_INCHES;

        resetDrive();

        left_drive.setTargetPosition(0);
        right_drive.setTargetPosition((int) round((WHEEL_CIRCUMFERENCE/(720/(degrees+5))) / COUNTS_PER_INCH));//10.00 is SLIGHTLY too much. 9.25 is too little. Like, it looks perfect, but after 4 rotations...
        //right_drive.setTargetPosition(100);


        //left_drive.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        //right_drive.setTargetPosition((int) (inches * COUNTS_PER_INCH));

        //left_drive.setPower(-tpower);
        left_drive.setPower(0);
        right_drive.setPower(tpower);

        setRunToPosition();

        waitForDrive();

        left_drive.setPower(0);
        right_drive.setPower(0);
    }

    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;
    //NOTE: to get heading, do degreesConversion()



    //Actual Code that runs during Auton
    @Override
    public void runOpMode() {

        left_drive = hardwareMap.get(DcMotor.class, "leftDrive");
        right_drive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(CRServo.class, "claw");

        //slide = hardwareMap.get(DcMotor.class, "slide");
        //webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        //blockPosBlue.blueInit(webcam);

//set target position, power, set run to position, wait for drive

        waitForStart();

        sleep(20000);

        arm.setPower(0.5);
        sleep(2000);
        arm.setPower(0);
        idle();
    }
}
