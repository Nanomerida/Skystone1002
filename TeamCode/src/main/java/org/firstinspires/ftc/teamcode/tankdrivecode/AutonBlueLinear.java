package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Methods.GeneralMethods;
import org.firstinspires.ftc.teamcode.Variables.Reference;
import org.firstinspires.ftc.teamcode.CRVuforia.VuforiaBlue;
import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;


import static java.lang.Math.abs;
import static java.lang.Math.round;

@Autonomous (name = "AutonBlueLinear", group = "Autonomous")

public class AutonBlueLinear extends LinearOpMode {


    public DcMotor  left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;
    public DcMotor  main_arm     = null;
    public DcMotor  slide = null;
    public DcMotor    claw   = null;

    private final double START_POSITION_CLAW       =  1.0 ; //starting pose of main claw servo



    private HardwareMapMain robot   = new HardwareMapMain();
    private GeneralMethods methods = new GeneralMethods();
    private ElapsedTime timer = new ElapsedTime(0);



    private void timeDelay(float delay){ //method for time delay
        timer.reset();
        while(timer.seconds() != delay ){
            boolean y = false; //i.e, do nothing
        }
    }

    public double degreesConversion(){
        double theta = this.angles.firstAngle;
        if(theta < 0) {
            theta += 360;
        }
        return theta;
    }

    private void resetDrive(){
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


    private void waitForDrive(){
        while(left_front_drive.isBusy() || left_back_drive.isBusy() || right_front_drive.isBusy() || right_back_drive.isBusy()){
            sleep(1);
        }
    }


    private void moveDrive(double power, float inches){
        resetDrive();

        left_front_drive.setPower(power);
        left_back_drive.setPower(power);
        right_front_drive.setPower(power);
        right_back_drive.setPower(power);

        left_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));

        setRunToPosition();

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
    }



    /**Make sure these measurments are correct*/
    static final double     ROBOT_WHEEL_DIST_INCHES = 8.5f;     // distance from center of robot to wheels
    static final double     COUNTS_PER_WHEEL_REV    = 96 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_MM       = 90 ;     // For figuring circumference
    static final float     COUNTS_PER_INCH         = 2.9452f;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /* Other Variables */
    public static final double degreesToRadians = 180.0 / Math.PI;
    private VuforiaBlue blockPosBlue = new VuforiaBlue();
    private Reference ref = new Reference();
    public static final double servoDegreesConst = 0.005;

    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;
    //NOTE: to get heading, do degreesConversion()


    @Override
    public void runOpMode() throws InterruptedException{

        int skystonePos = 4;

        blockPosBlue.blueInit(); //sets up vuforia


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
        telemetry.addData("Status","IMU Initialized, Current Heading %4d",
                degreesConversion());
        telemetry.update();



        //Initialize motors
        left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        //Arm
        slide = hardwareMap.get(DcMotor.class, "slide_motor");
        main_arm    = hardwareMap.get(DcMotor.class, "main_arm");
        claw = hardwareMap.get(DcMotor.class, "claw");


        /* Reset all encoders */
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        main_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);







        /*DO NOT DELETE!!!!!!!!!!!! If deleted, robot will automatically navigate to opponent's Capstone!!!!! */
        telemetry.addData("Say", "The Matrix is Ready");
        telemetry.addData("Glitches detected:", "0");
        telemetry.update();
        timeDelay(2.0f);


        telemetry.addData("Calculating Risk of Vuforia AI Taking Control .......", "....");
        telemetry.addData("Risk calculated:", ref.vuforiaRisk);
        telemetry.update();
        timeDelay(2.0f);

        waitForStart();


        //Steps
        slide.setPower(-1);
        moveDrive(1, 26.5f); // move forward to skystone
        waitForDrive();
        //each vuforia case should end at the same pos so they can be brought together for the next step.

// Degree constant is 3.111...
        //Grab stone
        claw.setPower(1);

        //next step
        turnDrive("ccw", .5, 90);
        waitForDrive();

        //and so on.
        moveDrive(1,65f);
        waitForDrive();

        turnDrive("cw", 0.5, 35);
        claw.setPower(-1); //open claw
        waitForDrive();

        turnDrive("ccw", 0.5, 35);
        waitForDrive();

        moveDrive(-1, 69f);
        waitForDrive();

        turnDrive("cw",0.5, 90);
        /*switch (testResult){ //vuforia
            case 0:
                //Pick up stone 1
                break;
            case 1:
                //Pick up stone 2
                break;
            case 2:
                //pick up stone 1
                break;
        } */

        //Pick Up Stone
        claw.setPower(1);

        turnDrive("ccw", .5, 90);
        waitForDrive();

        //and so on.
        moveDrive(1,69f);
        //claw.setPosition(90.0 * servoDegreesConst); //open claw
        claw.setPower(-1);
        waitForDrive();

        moveDrive(-1, 40f); // park under Skybridge







    }
}
