package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.CRVuforia.VuforiaBlue;
import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;
import org.firstinspires.ftc.teamcode.Variables.*;
import org.firstinspires.ftc.teamcode.Methods.*;

import static java.lang.Math.cos; //Ryan's Math Stuff
import static java.lang.Math.round;
import static java.lang.Math.sin;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import java.util.HashMap;

@Autonomous //Assuming this is right
//@Disabled
public class AutonomousCode extends OpMode {

    HardwareMapMain robot = new HardwareMapMain(); //need to define the hardware map
    VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file
    GoalLibraries library = new GoalLibraries();
    Reference constants = new Reference();
    GeneralMethods methods = new GeneralMethods();
    MecMoveProcedureStorage procedures = new MecMoveProcedureStorage();
    private ElapsedTime timer = new ElapsedTime(0);



    public DcMotor left_front_drive   = null;
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



    //IMU STUFF
    BNO055IMU imu;
    public Orientation angles;


    public double[] AbsolutePosition(double PrevX, double PrevY) {
        double[] PreviousPosition = {PrevX, PrevY};
        double[] CurrentPosition = new double[2];
        double[] XEncoderPosition = new double[2];
        double[] YEncoderPosition = new double[2];
        double ConvRate = (PI * 90) / 208076.8;
        double Heading = degreesConversion();
        double[] WeirdOrlandoMathsX = {sin(Heading), cos(Heading)};
        double[] WeirdOrlandoMathsY = {cos(Heading), sin(Heading)};
        double XClicks = 0.0d; //"encoders detected since previous iteration X"};
        double YClicks = 0.0d; //{"encoders detected since previous iteration Y"};
        for(int i = 0; i < 2; i++) {
            XEncoderPosition[i] = ConvRate * XClicks * WeirdOrlandoMathsX[i];
            YEncoderPosition[i] = ConvRate * YClicks * WeirdOrlandoMathsY[i];
        }
        for(int k = 0; k < 2; k++) {
            CurrentPosition[k] = XEncoderPosition[k] + YEncoderPosition[k] + PreviousPosition[k];
        }
        //something here to reset encoder readings.
        return CurrentPosition;
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

    public void moveDrivebyPower(double[] powers) { //method to move.
        left_front_drive.setPower(powers[0]);
        left_back_drive.setPower(powers[1]);
        right_front_drive.setPower(powers[2]);
        right_back_drive.setPower(powers[3]);
    }

    public void setRunToPosition(){
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRunUsingEncoder(){
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveDrivebyPos(String type, float inches){
        resetDrive();


        left_front_drive.setPower(mecanum.get(type) [0]);
        left_back_drive.setPower(mecanum.get(type) [1]);
        right_front_drive.setPower(mecanum.get(type) [2]);
        right_back_drive.setPower(mecanum.get(type) [3]);

        left_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));


        setRunUsingEncoder();

    }




    public void armMove(double armPower, double clawPower){ //convert to 1-0
        clawPower = methods.degreeServoConv(clawPower);
        main_arm.setPower(armPower);
        claw_level.setPosition(clawPower);
    }


    private boolean checkIfBusy(){
        boolean busy = true;
        if(!left_front_drive.isBusy() || !left_back_drive.isBusy() || !right_front_drive.isBusy() || !right_back_drive.isBusy()){
            busy = false;
        }
        return busy;

    }

    private boolean checkOtherBusy(){
        boolean busy = true;
        if(abs(timer.seconds() - 1.5) <= 0.03 ){ //check to see if 1.5 secs has passed since timer reset (servo command)
            busy = false;
        }
        if(!slide.isBusy() || !main_arm.isBusy()){
            busy = false;
        }
        return busy;
    }


    /** MUST CHANGE THIS ACCORDING TO DESIRED ROUTE */
    private double[][] goalLibrary = library.goalLibraryBBF;

        //0 is a position change in format {0, x, y}
        //1 is an angle change in format {1, angle, 0}
        //2 is the vision test for skystone in format {2, 0, 0}
        //3 is an arm change in format {3, arm power, claw head servo}
        //4 is an claw change in format {4, open(0)/close(1) claw, 0}
        //others as needed

    public static int stepNumber = 0;
    public static boolean newGoal = true; //variable if new goal is desired
    public static double[] previousPos = {0.00d, 0.00d}; //define starting position here. May change based on placement.
    public static final double servoDegreesConst = 0.005;
    static final float COUNTS_PER_INCH = 2.9452f;
    public static final float clawClosed = 120.0f;
    public static final float clawOpen = 180.0f;
    public static final String driveIdle = "IDLE";
    public static final String driveMoving = "MOVING";
    public static final String driveTurning = "TURNING";
    public static boolean vuforiaOn = false;
    private static int stonePos = 4;
    private static int stepVuf0 = 0;
    private static int stepVuf1 = 0;
    private static int stepVuf2 = 0;
    private static boolean otherThanDriveBusy = false;

    //Gets the HasMap of mecanum movement procedures
    private HashMap<String, int[]> mecanum = procedures.getMecanum();




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

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


    }


         /* NOTE: Maybe put some more of these methods in a class?*/

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        claw_level.setPosition(START_POSITION_CLAW_LEVELER);
        claw.setPosition(START_POSITION_CLAW);
        claw_rotate.setPosition(START_POSITION_CLAW_ROTATER);


        /*DO NOT DELETE!!!!!!!!!!!! If deleted, robot will automatically navigate to opponent's Capstone!!!!! */
        telemetry.addData("Glitches in MATRIX detected:", 0);
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Telemetry.Item driveStatus = telemetry.addData("Drive Base Status:", driveIdle); //drive status
        Telemetry.Item armStatus = telemetry.addData("Arm Motor Status:", "IDLE");
        Telemetry.Item clawLevelStatus = telemetry.addData("Claw Level Servo Status:", "IDLE");
        Telemetry.Item clawStatus = telemetry.addData("Claw Servo Status:", "IDLE");
        Telemetry.Item visionStatus = telemetry.addData("Vision Testing Status:", "DISABLED"); //first item
        Telemetry.Item stepNumb = telemetry.addData("Current Step Number", stepNumber);
        Telemetry.Item currentHeading = telemetry.addData("Current Heading:", degreesConversion());
        telemetry.update();

        int goalType = (int) goalLibrary[stepNumber][0]; //Setting goal each time
        newGoal = false;

        boolean busy = checkIfBusy();
        otherThanDriveBusy = checkOtherBusy();


        if (vuforiaOn = true) {
            if (stonePos == 0) {
                if (stepVuf0 == 0 && !busy) {
                    moveDrivebyPos("strafeL", 8.0f);
                    stepVuf0 = 1;
                }
                if(stepVuf0 == 1 && !busy){
                    claw.setPosition(90.0);
                    stepVuf0 = 2;
                }
                if(stepVuf0 == 2 && !busy){
                    //move arm
                }


            }
        }


        switch (goalType) {

            /* Position Change */
            case 0:
                if(otherThanDriveBusy) {
                    double[] actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
                    boolean goalReachedPos = methods.GoalCheckPos(actualPos[0], goalLibrary[stepNumber][1], actualPos[1], goalLibrary[stepNumber][2]); //check if we are at position
                    if (goalReachedPos) {
                        newGoal = true;
                    } else {
                        double[] motorPowerPos = methods.PositionChange(actualPos[0], goalLibrary[stepNumber][1], actualPos[2], goalLibrary[stepNumber][2]);
                        moveDrivebyPower(motorPowerPos);
                    }
                }
                break;


            /* Angle Change */
            case 1:
                if(otherThanDriveBusy) {

                    boolean goalReachedAngle = methods.GoalCheckAngle(goalLibrary[stepNumber][1]); //check if we are at angle.
                    if (goalReachedAngle) {
                        double[] motorPowerAngle = methods.AngleChange(goalLibrary[stepNumber][1]);
                        moveDrivebyPower(motorPowerAngle);
                    }
                }
                break;

            /* Vuforia.java :( ugh... */
            case 2:
                if (!vuforiaOn) {

                    visionStatus.setValue("ENABLED; SEARCHING");
                    telemetry.update();
                    if (goalLibrary[stepNumber][1] == 1) {
                        stonePos = blockPosBlue.visionTest();
                    }
                    vuforiaOn = true;
                }


                switch (stonePos) {
                    case 0: //first position (left, towards skybridge)
                        visionStatus.setValue("SKYSTONE: TOWARDS BRIDGE");
                        telemetry.update();
                        //go there;
                        //grab thing;
                        //go back;
                        goalLibrary[12][1] = 0;//change steps in library because we know where other Skystone is
                        goalLibrary[12][2] = 0;
                        break;
                    case 1: //second position (center)
                        visionStatus.setValue("SKYSTONE: CENTER");
                        telemetry.update();
                        //go there;
                        //grab thing;
                        //go back;
                        goalLibrary[13][0] = 1;
                        goalLibrary[13][1] = 0; //change steps in library because we know where other Skystone is
                        break;
                    case 2: //third position (right, towards wall)
                        visionStatus.setValue("SKYSTONE: TOWARDS WALL");
                        telemetry.update();
                        //go there;
                        //grab thing;
                        //go back;
                        goalLibrary[14][1] = 0; //change steps in library because we know where other Skystone is
                        goalLibrary[14][2] = 0;
                        break;
                }
                newGoal = true;
                break;

            /* Arm Change */
            case 3:
                if(!busy) {
                    double clawLevelPower =  0; //armClawPower(goalLibrary[stepNumber][1]);
                    armMove(goalLibrary[stepNumber][1], clawLevelPower);
                }

                break;

            case 4: //claw: open is 0, closed is 1
                if (goalLibrary[stepNumber][1] == 0) { //open claw
                    clawStatus.setValue("OPENING");
                    telemetry.update();
                    timer.reset();
                    claw.setPosition(clawOpen);
                } else { //close claw
                    clawStatus.setValue("CLOSING");
                    telemetry.update();
                    timer.reset();
                    claw.setPosition(clawClosed);
                }
                clawStatus.setValue("IDLE");
                telemetry.update();
                break;

            case 5:
                slide.setTargetPosition(120);
                break;
        }
        if (newGoal) {
            stepNumber++;
            stepNumb.setValue(stepNumber);
        }
        telemetry.update();
    }



        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }
}

