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


import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.round;

@Autonomous (name = "BlueLoadingZoneTank", group = "Autonomous")

public class BlueLoadingZone extends LinearOpMode {


    public DcMotor left_drive = null;
    public DcMotor right_drive = null;
    //public Servo    claw   = null;
    //public DcMotor slide = null;
    //public DcMotor main_arm = null;
    //public WebcamName webcam = null;





    static final double ROBOT_WHEEL_DIST_INCHES = 5.5;     // distance from center of robot to wheels
    static final double COUNTS_PER_WHEEL_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 3.75;     // For figuring circumference
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * PI;
    static final double COUNTS_PER_INCH = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE;





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
            sleep(400);
        }
    }

    private void moveDrive(float power, float inches) {

        resetDrive();

        left_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));
        right_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));

        left_drive.setPower(power);
        right_drive.setPower(power);

        setRunToPosition();

        waitForDrive();

        left_drive.setPower(0);
        right_drive.setPower(0);
    }

   /* public void turnDrive(String direction, double tpower, double degrees) {

        String cc = "cw";

        double inches = (degrees * degreesToRadians) * ROBOT_WHEEL_DIST_INCHES;
        float speedLeft = 1.0f;
        float speedRight = 1.0f;

        if(direction.equals(cc)){
            speedRight = -1.0f;
            speedLeft = 0f;
        }
        else{
            speedLeft = -1.0f;
            speedRight = 0f;
        }

        resetDrive();

        left_drive.setTargetPosition((int) round(degrees / COUNTS_PER_TICK));
        right_drive.setTargetPosition((int) round(degrees / COUNTS_PER_TICK));

        //set desired power
        left_drive.setPower(tpower * speedLeft);
        right_drive.setPower(tpower * speedRight);
        setRunToPosition();

        waitForDrive();

        left_drive.setPower(0);
        right_drive.setPower(0);
    }*/

    public void turnDrive(double tpower, double degrees) {
        resetDrive();

        double inches = (degrees * degreesToRadians) * ROBOT_WHEEL_DIST_INCHES;

        left_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));
        //right_drive.setTargetPosition((int) round(WHEEL_CIRCUMFERENCE/(720/(degrees+9.85)) / COUNTS_PER_TICK));//10.00 is SLIGHTLY too much. 9.25 is too little. Like, it looks perfect, but after 4 rotations...
        right_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));

        left_drive.setPower(-tpower);
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

        left_drive  = hardwareMap.get(DcMotor.class, "leftDrive");
        right_drive = hardwareMap.get(DcMotor.class, "rightDrive");
        //main_arm = hardwareMap.get(DcMotor.class, "main_arm");
        //slide = hardwareMap.get(DcMotor.class, "slide");
        //webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        //blockPosBlue.blueInit(webcam);

//set target position, power, set run to position, wait for drive


        waitForStart();

        moveDrive(0.7f,20);
        turnDrive(0.3f,90);
        moveDrive(0.7f,14);

/*
        int skystonePos;
        skystonePos = blockPosBlue.visionTest();
        switch (skystonePos) {
            case 0:
                moveDrive(1f,14);
                turnDrive(0.5f, 90);
                //arm.setPower(1);
                //claw.setPosition(-0.5);
                break;
            case 1:
                break;
            case 2:
                break;
        }
*/
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


















