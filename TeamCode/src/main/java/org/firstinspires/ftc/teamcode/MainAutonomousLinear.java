package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Variables.*;
import org.firstinspires.ftc.teamcode.Methods.*;

import static java.lang.Math.cos; //Ryan's Math Stuff
import static java.lang.Math.round;
import static java.lang.Math.sin;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import java.util.HashMap;
import java.util.ArrayList;


@Autonomous(name = "Main Auto Linear", group = "Autonomous")
@Disabled

public class MainAutonomousLinear extends LinearOpMode {



    VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file
    VuforiaRed blockPosRed = new VuforiaRed(); //creates an instance of the vuforia red side file
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

    //IMU STUFF
    private BNO055IMU imu;
    private Orientation angles;

    private double[] AbsolutePosition(double PrevX, double PrevY) {
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


    private double degreesConversion(){
        double theta = this.angles.firstAngle;
        if(theta < 0) {
            theta += 360;
        }
        return theta;
    }

    private void resetDrive(){
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        /*left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); */
    }

    private void moveDrivebyPower(double[] powers) { //method to move.
        int x = 0;
        for(DcMotor motor : driveMotors){
            motor.setPower(powers[x]);
            x++;
        }

        /* left_front_drive.setPower(powers[0]);
        left_back_drive.setPower(powers[1]);
        right_front_drive.setPower(powers[2]);
        right_back_drive.setPower(powers[3]);

         */
    }
    private void stopDrive(){
        for(DcMotor motor : driveMotors){
            motor.setPower(0);
        }
    }

    public void setRunToPosition(){
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        /*left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION); */
    }

    public void setRunUsingEncoder(){
        stopDrive();
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        /*left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }

    private void moveDrivebyPos(String type, float inches){
        //set the
        resetDrive();
        setRunToPosition();


        left_front_drive.setPower(mecanum.get(type) [0]);
        left_back_drive.setPower(mecanum.get(type) [1]);
        right_front_drive.setPower(mecanum.get(type) [2]);
        right_back_drive.setPower(mecanum.get(type) [3]);

        left_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        left_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        right_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        right_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));



    }


    private boolean checkIfBusy(){
        boolean busy = true;
        if(!left_front_drive.isBusy() || !left_back_drive.isBusy() || !right_front_drive.isBusy() || !right_back_drive.isBusy()){
            busy = false;
        }
        return busy;

    }

    private void waitForDrive(){
        while(!left_front_drive.isBusy() || !left_back_drive.isBusy() || !right_front_drive.isBusy() || !right_back_drive.isBusy()){
            //literally do nothing
        }
    }

    private double[][] goalLibrary = library.goalLibraryBBF;

    //0 is a position change in format {0, x, y}
    //1 is an angle change in format {1, angle, 0}
    //2 is the vision test for skystone in format {2, 0, 0}
    //3 is an arm change in format {3, arm power, claw head servo}
    //4 is an claw change in format {4, open(0)/close(1) claw, 0}
    //others as needed


    public static int stepNumber = 0;
    public static double[] previousPos = {0.00d, 0.00d}; //define starting position here. May change based on placement.
    public static final double servoDegreesConst = 0.005;
    private final float COUNTS_PER_INCH = 2.9452f; /**Needs to be updated!!!!!!! */
    private static int stonePos = -1;
    private static boolean goalReachedPos = false;
    private static boolean goalReachedAngle = false;


    //Gets the HasMap of mecanum movement procedures
    private HashMap<String, int[]> mecanum = procedures.getMecanum();

    //Creates list to hold motors
    private ArrayList<DcMotor> driveMotors = new ArrayList<DcMotor>();




    @Override
    public void runOpMode() {

        //Initialize motors
        left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        //Arm
        slide = hardwareMap.get(DcMotor.class, "slide_motor");
        main_arm    = hardwareMap.get(DcMotor.class, "main_arm");




        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        main_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //adds motors to ArrayList
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




        waitForStart();



        /*DO NOT DELETE!!!!!!!!!!!! If deleted, robot will automatically navigate to opponent's Capstone!!!!! */
        telemetry.addData("Glitches in MATRIX detected:", 0);
        telemetry.update();


        //HERE IS WHERE WE CAN PUT THE ORIGINAL OP MODE MECANUM CODE LOOP().
        //SOME THINGS CAN BE CHANGED DUE TO A LINEAR OP SETTING


        while(opModeIsActive()){


            int goalType = (int) goalLibrary[stepNumber][0]; //Setting goal each time




            //OLD VUFORIA CODE, needs changed


            //OLD SWITCH STATEMENT
            switch(goalType) {


                /* Position Change */
                case 0:
                    do {
                        double[] actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
                        goalReachedPos = methods.GoalCheckPos(actualPos[0], goalLibrary[stepNumber][1], actualPos[1], goalLibrary[stepNumber][2]); //check if we are at position
                        if (!goalReachedPos) {
                            moveDrivebyPower(methods.PositionChange(actualPos[0], goalLibrary[stepNumber][1], actualPos[2], goalLibrary[stepNumber][2]));
                        }
                    }
                    while(!goalReachedPos);
                    stopDrive();
                    break;



                /* Angle Change */
                case 1:
                    do {
                        boolean goalReachedAngle = methods.GoalCheckAngle(goalLibrary[stepNumber][1]); //check if we are at angle.
                        if (goalReachedAngle) {
                            moveDrivebyPower(methods.AngleChange(goalLibrary[stepNumber][1]));
                        }
                    }
                    while(!goalReachedAngle);
                    stopDrive();
                    break;




                /* Vuforia.java :( ugh... */
                case 2:
                        telemetry.addData("Vision:", "Searching");
                        telemetry.update();
                        if (goalLibrary[stepNumber][1] == 1) {
                            stonePos = blockPosBlue.visionTest();
                        } else {
                            stonePos = blockPosRed.visionTest();
                        }

                        if(goalLibrary[stepNumber][1] == 1){ //for blue side
                            switch(stonePos){
                                case 0: //bridge
                                    moveDrivebyPos("strafeL", 6.5f); //NEED TO REDUCE POWER FOR STRAFE
                                    waitForDrive();

                                    //open claw, move arm up, etc

                                    moveDrivebyPos("forward", 12.5f);
                                    waitForDrive();

                                    //close claw

                                    moveDrivebyPos("backward", 12.5f);
                                    waitForDrive();

                                    goalLibrary[12][1] = 0;//change steps in library because we know where other Skystone is
                                    goalLibrary[12][2] = 0;

                                    break;
                                case 1: //center
                                    //open claw, move arm up, etc

                                    moveDrivebyPos("forward", 12.5f);
                                    waitForDrive();

                                    //close claw

                                    moveDrivebyPos("backward", 12.5f);
                                    waitForDrive();

                                    goalLibrary[13][0] = 1;
                                    goalLibrary[13][1] = 0; //change steps in library because we know where other Skystone is

                                    break;
                                case 2: //wall
                                    //move there
                                    //get stone

                                    goalLibrary[14][1] = 0; //change steps in library because we know where other Skystone is
                                    goalLibrary[14][2] = 0;

                                    break;

                            }

                        }

                        else{
                            switch (stonePos){
                                case 0:
                                    //move there


                                    goalLibrary[12][1] = 0;//change steps in library because we know where other Skystone is
                                    goalLibrary[12][2] = 0;
                                    break;
                                case 1:
                                    //move there

                                    goalLibrary[13][0] = 1;
                                    goalLibrary[13][1] = 0; //change steps in library because we know where other Skystone is
                                    break;
                                case 2:
                                    //move there

                                    goalLibrary[14][1] = 0; //change steps in library because we know where other Skystone is
                                    goalLibrary[14][2] = 0;
                                    break;

                            }
                        }
                    break;

                    //MAYBE INTAKE HERE:
                case 3:
                    //do something
                    break;


                //SLIDE MOTOR:
                case 4:
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setTargetPosition(120);
                    slide.setPower(0.4);
                    while(slide.isBusy()){
                        sleep(1);
                    }
                    break;
            }
            stepNumber++;
            telemetry.update();




        }


    }
}
