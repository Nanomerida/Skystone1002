package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Methods.*;
import org.firstinspires.ftc.teamcode.CRVuforia.*;
import org.firstinspires.ftc.teamcode.Variables.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static java.lang.Math.round;


@Autonomous(name="MecaunmAutonBlue", group = "Mecanum")
public class SimpleAutonBlue extends LinearOpMode {

    VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file
    GeneralMethods methods = new GeneralMethods();
    private ElapsedTime refreshTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);




     DcMotor left_front_drive   = null;
     DcMotor  left_back_drive  = null;
     DcMotor  right_front_drive = null;
     DcMotor  right_back_drive = null;
     DcMotor arm = null;
     CRServo claw = null;
     public WebcamName webcam = null;

    public BNO055IMU imu;
    private Orientation angles;


    private void MoveAngle(double power, double angleGoal){
        //Use odometry to rotate
        do {
            boolean goalReachedAngle = methods.GoalCheckAngle(angleGoal, degreesConversion()); //check if we are at angle.
            if (goalReachedAngle) moveDriveByPower(methods.AngleChange(angleGoal));
        }
        while(!goalReachedAngle);
        stopDrive();
    }

    private void moveDriveByPos(float power, float inches) {

        resetDrive();

        //left_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));
        //right_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));

        left_front_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));

        moveDriveByPower(new double[] {power, power, power, power});


        setRunToPosition();

        waitForDrive();

        moveDriveByPower(new double[] {0, 0, 0, 0});
    }

    private void strafeL(double power, double inches){

        resetDrive();

        //left_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));
        //right_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));

        left_front_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));

        moveDriveByPower(new double[] {-power, power, power, -power}); //strafe Left


        setRunToPosition();

        waitForDrive();

        moveDriveByPower(new double[] {0, 0, 0, 0});
    }
    private void strafeR(double power, double inches){

        resetDrive();

        //left_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));
        //right_drive.setTargetPosition((int) round(inches * COUNTS_PER_INCH));

        left_front_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition((int) round((inches/12) / COUNTS_PER_INCH));

        moveDriveByPower(new double[] {power, -power, -power, power}); //strafe Left


        setRunToPosition();

        waitForDrive();

        moveDriveByPower(new double[] {0, 0, 0, 0});
    }

    private void moveDriveByPower(double[] powers) { //method to move.
        int x = 0;
        for (DcMotor motor : driveMotors) {
            motor.setPower(powers[x]);
            x++;
        }
    }

    private void setRunToPosition(){
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    private void waitForDrive(){
        while(left_front_drive.isBusy() ||left_back_drive.isBusy() || right_front_drive.isBusy() || right_back_drive.isBusy()){
            idle();
        }
    }

    private void stopDrive(){
        for(DcMotor motor : driveMotors){
            motor.setPower(0);
        }
    }

    private void resetDrive(){
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private double degreesConversion(){
        while(refreshTimer.milliseconds() < 3){sleep(1);}
        refreshTimer.reset();
        double theta = AngleUnit.normalizeDegrees(this.angles.firstAngle);
        if(theta < 180) theta += 180;
        else theta -= 180;
        return theta;
    }

    private ArrayList<DcMotor> driveMotors = new ArrayList<DcMotor>();

    private static boolean goalReachedAngle = false;
    static final double COUNTS_PER_WHEEL_REV = 2114.5936;
    static final double WHEEL_DIAMETER_INCHES = 3;     // For figuring circumference
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * PI;
    static final double COUNTS_PER_INCH =  ((WHEEL_CIRCUMFERENCE)/(COUNTS_PER_WHEEL_REV));

    @Override
    public void runOpMode(){

        left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);

        arm = hardwareMap.get(DcMotor.class, "arm");

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        claw = hardwareMap.get(CRServo.class, "claw");

        blockPosBlue.blueInit(webcam);

        driveMotors.add(left_front_drive);
        driveMotors.add(left_back_drive);
        driveMotors.add(right_front_drive);
        driveMotors.add(right_back_drive);


        // MORE IMU STUFF

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        telemetry.addData("Status:", "Ready!");
        telemetry.update();
        waitForStart();


        sleep(5000);

        claw.setPower(-0.3);
        sleep(1000);
        claw.setPower(0);

        moveDriveByPos(0.5f,29.6f);

        //Vuforia

        int skystonePos = blockPosBlue.visionTest();
        if (skystonePos == 4){
            skystonePos = 1;
            telemetry.addData("Vuforia Error!", "Defaluting!");
            telemetry.update();
        }

        switch (skystonePos){
            case 0:
                strafeL(0.3, 8);
                sleep(400);

                moveDriveByPos(0.3f, 19);
                sleep(400);

                claw.setPower(0.5);
                sleep(3000);
                claw.setPower(0.1);

                moveDriveByPos(0.3f, -19);
                sleep(400);

                MoveAngle(0.3, -90);
                sleep(400);

                moveDriveByPos(0.5f, 40);
                sleep(300);

                claw.setPower(-0.7);
                sleep(2000);
                claw.setPower(0);
                break;
            case 1:
                moveDriveByPos(0.3f, 19);
                sleep(400);

                claw.setPower(0.5);
                sleep(3000);
                claw.setPower(0.1);

                moveDriveByPos(0.3f, -19);
                sleep(400);

                MoveAngle(0.3, -90);
                sleep(400);

                moveDriveByPos(0.5f, 45);
                sleep(300);

                claw.setPower(-0.7);
                sleep(2000);
                claw.setPower(0);
                break;
            case 2:
                strafeR(0.3, 8);
                sleep(400);

                moveDriveByPos(0.3f, 19);
                sleep(400);

                claw.setPower(0.5);
                sleep(3000);
                claw.setPower(0.1);

                moveDriveByPos(0.3f, -19);
                sleep(400);

                MoveAngle(0.3, -90);
                sleep(400);

                moveDriveByPos(0.5f, 50);
                sleep(300);

                claw.setPower(-0.7);
                sleep(2000);
                claw.setPower(0);
                break;
        }

        moveDriveByPos(-40f, -20);

        idle();
    }
}
