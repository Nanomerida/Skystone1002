package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous (name = "AutonRedLinear", group = "Autonomous")

public class AutonRedLinear extends LinearOpMode {


    private HardwareMapMain robot   = new HardwareMapMain();
    private GeneralMethods methods = new GeneralMethods();
    private ElapsedTime timer = new ElapsedTime(0);


    public DcMotor  left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;
    public DcMotor  main_arm     = null;
    public DcMotor  slide = null;
    public Servo claw_level    = null;
    public Servo    claw   = null;
    public Servo    claw_rotate = null;

    private final double START_POSITION_CLAW       =  1.0 ; //starting pose of main claw servo
    private final double START_POSITION_CLAW_LEVELER = 0.0; //starting pose of the claw leveler
    private final double START_POSITION_CLAW_ROTATER = 0.0;

    HardwareMap hwMap           =  null;


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

    /*Moving claw to keep up with arm movement */

    public double armClawPower(double armPower){ //write orlando's math here for arm to claw_leveler power.
        telemetry.update();
        return armPower;
    }

    /* Arm */
    public void armMove(int armPower, double clawPower){ //convert degrees to Servo powers
        clawPower = methods.degreeServoConv(clawPower);
        main_arm.setPower(0.5);
        main_arm.setTargetPosition(armPower);
        claw_level.setPosition(clawPower);
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
    static final double     WHEEL_DIAMETER_MM       = 96 ;     // For figuring circumference
    static final float     COUNTS_PER_INCH         = 2.9452f;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /* Other Variables */
    public static final double degreesToRadians = 180.0 / Math.PI;
    private VuforiaRed blockPosRed = new VuforiaRed();
    private Reference ref = new Reference();
    public static final double servoDegreesConst = 0.005;

    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;
    //NOTE: to get heading, do degreesConversion()

    @Override
    public void runOpMode() throws InterruptedException{

        int skystonePos = 4;

        blockPosRed.redInit(); //sets up vuforia


        // MORE IMU STUFF

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Status","IMU Initialized, Current Heading %4d",
                degreesConversion());
        telemetry.update();




        //Initialize motors
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


        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        main_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        slide.setPower(0);
        main_arm.setPower(0);







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

        claw_level.setPosition(START_POSITION_CLAW_LEVELER);
        claw.setPosition(START_POSITION_CLAW);
        claw_rotate.setPosition(START_POSITION_CLAW_ROTATER);


        //Steps
        moveDrive(0.8, 26.5f); // move forward to skystone
        waitForDrive();

     /*   int testResult = blockPosRed.visionTest();
        switch (testResult){ //vuforia
            case 0: //towards bridge
                //move there
                main_arm.setPower(0.4);
                main_arm.setTargetPosition(280); //90 degrees down
                main_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                claw_level.setPosition(90.0 * servoDegreesConst); //claw level at 90 to match arm
                while(main_arm.isBusy()) {
                    sleep(1);
                }//wait for arm
                claw.setPosition(90.0 * servoDegreesConst); //open claw

                moveDrive(0.5, 6.5f);
                waitForDrive();

                skystonePos = 0;
                break;
            case 1: //center
                //move there
                main_arm.setPower(0.4);
                main_arm.setTargetPosition(280); //90 degrees down
                main_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                claw_level.setPosition(90.0 * servoDegreesConst); //claw level at 90 to match arm
                while(main_arm.isBusy()) {
                    sleep(1);
                }//wait for arm
                claw.setPosition(90.0 * servoDegreesConst); //open claw

                turnDrive("ccw", 0.3, 20.0f);
                waitForDrive();

                moveDrive(0.5, 6.5f);
                waitForDrive();

                skystonePos = 1;
                break;
            case 2: //towards wall
                main_arm.setPower(0.4);
                main_arm.setTargetPosition(280); //90 degrees down
                main_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                claw_level.setPosition(90.0 * servoDegreesConst); //claw level at 90 to match arm
                while(main_arm.isBusy()) {
                    sleep(1);
                }//wait for arm
                claw.setPosition(90.0 * servoDegreesConst); //open claw

                turnDrive("cc", 0.3, 20);
                waitForDrive();

                moveDrive(0.5, 6.5f);
                waitForDrive();

                skystonePos = 2;
                break;
        }*/
        //each vuforia case should end at the same pos so they can be brought together for the next step.

        // Degree constant is 3.111...
        //Grab stone
        claw.setPosition(0);
        main_arm.setPower(0.4);
        main_arm.setTargetPosition(62);
        claw_level.setPosition(1);
        claw.setPosition(0.5);

        //next step
        turnDrive("cw", .5, 90);
        waitForDrive();

        moveDrive(1,65f);
        waitForDrive();

        turnDrive("ccw", 0.5, 35);
        waitForDrive();

        claw.setPosition(90.0 * servoDegreesConst); //open claw to release stone
        sleep(1000);

        turnDrive("cw", 0.5, 35);
        waitForDrive();

        moveDrive(-1, 69f);
        waitForDrive();

        turnDrive("ccw",0.5, 90);
        waitForDrive();

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
        }*/

        //Pick Up Stone
        claw.setPosition(0);
        claw.setPosition(0.5);

        turnDrive("ccw", .5, 90);
        waitForDrive();

        //and so on.
        moveDrive(1,69f);
        //claw.setPosition(90.0 * servoDegreesConst); //open claw
        claw.setPosition(1);
        waitForDrive();

        moveDrive(-1, 40f); // park under Skybridge
    }



}
