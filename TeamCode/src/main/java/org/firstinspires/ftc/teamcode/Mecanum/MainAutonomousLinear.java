package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Variables.*;
import org.firstinspires.ftc.teamcode.Methods.*;
import org.firstinspires.ftc.teamcode.CRVuforia.*;

import static java.lang.Math.cos; //Ryan's Math Stuff
import static java.lang.Math.round;
import static java.lang.Math.sin;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;



import java.util.HashMap;
import java.util.ArrayList;


@Autonomous(name = "Main Auto Linear", group = "Autonomous")
@Disabled

public class MainAutonomousLinear extends LinearOpMode {

    VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file
    Reference constants = new Reference();
    GeneralMethods methods = new GeneralMethods();
    MecMoveProcedureStorage procedures = new MecMoveProcedureStorage();
    private ElapsedTime refreshTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private RobotState robotState = new RobotState();




    public DcMotor left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;
    //public DcMotor  left_y_encoder = null;
    //public DcMotor  right_y_encoder = null;
    //public DcMotor  x_encoder = null;
    public DcMotor  lift = null;
    public CRServo intake_wheel_left = null;
    public CRServo intake_wheel_right = null;
    public WebcamName webcam = null;
    ExpansionHubEx expansionHub4;
    RevBulkData bulkData;
    ExpansionHubMotor left_y_encoder, right_y_encoder, x_encoder = null;



    //Choose a goal library. Options are:
    //Blue side, skystone : "blue", "skystone"
    //Blue side, foundation: "blue", "foundation"
    //Red side, skystone: "red", "skystone"
    //Red side, foundation: "red", "foundation"
    private final String[] taskChoose = {"blue", "skystone"};

    //IMU STUFF
    public BNO055IMU imu;
    private Orientation angles;


    private double[] AbsolutePosition(double PrevX, double PrevY) {
        double[] PreviousPosition = {PrevX, PrevY};
        double[] CurrentPosition = new double[2];
        double[] XEncoderPosition = new double[2];
        double[] YEncoderPosition = new double[2];
        double ConvRate = (PI * 90) / 208076.8; /** Change this */
        double theta = degreesConversion();
        double[] WeirdOrlandoMathsX = {sin(theta), cos(theta)};
        double[] WeirdOrlandoMathsY = {cos(theta), sin(theta)};
        bulkData = expansionHub4.getBulkInputData();
        int XClicks = ticksX();
        int YClicks = ((ticksLeftY() + ticksRightY()) / 2);
        for(int i = 0; i < 2; i++) {
            XEncoderPosition[i] = ConvRate * XClicks * WeirdOrlandoMathsX[i];
            YEncoderPosition[i] = ConvRate * YClicks * WeirdOrlandoMathsY[i];
        }
        for(int k = 0; k < 2; k++) {
            CurrentPosition[k] = XEncoderPosition[k] + YEncoderPosition[k] + PreviousPosition[k];
        }
        resetEncoders();
        return CurrentPosition;
    }


    private int ticksLeftY(){
        int deltaTicks = (previousTicksYLeft - bulkData.getMotorCurrentPosition(left_y_encoder));
        return deltaTicks;
    }

    private int ticksRightY(){
        int deltaTicks = (previousTicksYRight - bulkData.getMotorCurrentPosition(right_y_encoder));
        return deltaTicks;
    }

    private int ticksX(){
        int deltaTicks = (previousTicksX - bulkData.getMotorCurrentPosition(x_encoder));
        return deltaTicks;
    }

    private void resetEncoders(){
        previousTicksYLeft = bulkData.getMotorCurrentPosition(left_y_encoder);
        previousTicksYRight = bulkData.getMotorCurrentPosition(right_y_encoder);
        previousTicksX = bulkData.getMotorCurrentPosition(x_encoder);
    }


    private double degreesConversion(){
        while(refreshTimer.milliseconds() < 3){sleep(1);}
        refreshTimer.reset();
        double theta = this.angles.firstAngle;
        if(theta < 0) theta += 360;
        if(redSide){
            if(theta < 0) theta += 180;
            else theta -= 180;
        }
        return theta;
    }

    private void intake(){
        robotState.setIntakeState(RobotState.IntakeState.ACTIVE);
        showTelemetry.updateTelem.updateTelemetry();

        intake_wheel_left.setPower(1);
        intake_wheel_right.setPower(1);
        sleep(2000);
        intake_wheel_left.setPower(0);
        intake_wheel_right.setPower(0);

        robotState.setIntakeState(RobotState.IntakeState.IDLE);
        showTelemetry.updateTelem.updateTelemetry();
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


    private void waitForDrive(){
        while(!left_front_drive.isBusy() || !left_back_drive.isBusy() || !right_front_drive.isBusy() || !right_back_drive.isBusy()){
            //literally do nothing
        }
    }


    private void MovePosition(){
        robotState.setDriveState(RobotState.DriveState.MOVING);
        showTelemetry.updateTelem.updateTelemetry();
        //Use odometry to move
        do {
            double[] actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
            goalReachedPos = methods.GoalCheckPos(actualPos[0], goalLibrary[stepNumber][1], actualPos[1], goalLibrary[stepNumber][2]); //check if we are at position
                moveDrivebyPower(methods.PositionChange(actualPos[0], goalLibrary[stepNumber][1], actualPos[2], goalLibrary[stepNumber][2]));
            previousPos[0] = actualPos[0];
            previousPos[1] = actualPos[1];
        }
        while(!goalReachedPos);
        robotState.setDriveState(RobotState.DriveState.IDLE);
        showTelemetry.updateTelem.updateTelemetry();
        stopDrive();
    }

    private void MoveOdomPosition(double GoalX, double GoalY){

        //Use odometry to move to a given position
        do {
            double[] actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
            goalReachedPos = methods.GoalCheckPos(actualPos[0], GoalX, actualPos[1], GoalY); //check if we are at position
            if (!goalReachedPos) {
                moveDrivebyPower(methods.PositionChange(actualPos[0], GoalX, actualPos[2], GoalY));
            }
            previousPos[0] = actualPos[0];
            previousPos[1] = actualPos[1];
        }
        while(!goalReachedPos);
        stopDrive();
    }

    private void MoveAngle(){
        robotState.setDriveState(RobotState.DriveState.MOVING);
        showTelemetry.updateTelem.updateTelemetry();
        //Use odometry to rotate
        do {
            boolean goalReachedAngle = methods.GoalCheckAngle(goalLibrary[stepNumber][1]); //check if we are at angle.
            if (goalReachedAngle) {
                moveDrivebyPower(methods.AngleChange(goalLibrary[stepNumber][1]));
            }
        }
        while(!goalReachedAngle);
        robotState.setDriveState(RobotState.DriveState.IDLE);
        showTelemetry.updateTelem.updateTelemetry();
        stopDrive();
    }



    //0 is a position change in format {0, x, y}
    //1 is an angle change in format {1, angle, 0}
    //2 is the vision test for skystone in format {2, 0, 0}
    //3 is an arm change in format {3, arm power, claw head servo}
    //4 is an claw change in format {4, open(0)/close(1) claw, 0}
    //others as needed


    private static int stepNumber = 1;
    private static double[] previousPos; //define starting position here. May change based on placement.
    private static int stonePos;
    private static int previousTicksYLeft = 0;
    private static int previousTicksYRight = 0;
    private static int previousTicksX = 0;
    private static boolean goalReachedPos = false;
    private static boolean goalReachedAngle = false;
    private static boolean redSide;



    //Gets the HasMap of mecanum movement procedures
    private HashMap<String, float[]> mecanum = procedures.getMecanum();

    //Creates list to hold motors
    private ArrayList<DcMotor> driveMotors = new ArrayList<DcMotor>();

    private ShowTelemetry showTelemetry = new ShowTelemetry(this, robotState);


    private double[][] goalLibrary;




    @Override
    public void runOpMode() {

        telemetry.addData("Start:", "telem");
        telemetry.update();


        //Load Goal Library.
        showTelemetry.telemetryMessage("Goal Library:", ("color:" + taskChoose[0] + "task" + taskChoose[1]));
        showTelemetry.updateTelem.updateTelemetry();
        try {
            GoalLibraries library = new GoalLibraries("blue", "skystone");
            showTelemetry.telemetryMessage("Goal Library:", "Successfully Loaded");
            showTelemetry.updateTelem.updateTelemetry();
            goalLibrary = library.choosenLibrary;
        }
        //Runs a scuffed opMode if no goal library can be loaded
        catch(NoFoundGoalLibraryException e) {
            showTelemetry.telemetryMessage("Someone misspelled the word:", e.toString());
            showTelemetry.updateTelem.updateTelemetry();

            waitForStart();

            while(opModeIsActive()){
                idle();
            }
        }
        previousPos[0] = goalLibrary[0][0]; //sets the starting position for the robot
        previousPos[1] = goalLibrary[0][1];
        if(goalLibrary[0][2] == 1) redSide = true;



        //Initialize motors
        left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        //Lift
        lift = hardwareMap.get(DcMotor.class, "slide_motor");

        //Expansion Hub with encoders
        expansionHub4 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");

        //External encoders
        left_y_encoder = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "left_y_encoder");
        right_y_encoder = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "right_y_encoder");
        x_encoder = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "x_encoder");
        
        //Intake wheels
        intake_wheel_left = hardwareMap.get(CRServo.class, "intake_wheel_left");
        intake_wheel_right = hardwareMap.get(CRServo.class, "intake_wheel_right");

        //Webcam
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //Initialize vuforia with webcam
        blockPosBlue.blueInit(webcam);




        //Set up the drive base
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reset the lift
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //reset the external encoders
        left_y_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_y_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        x_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_y_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_y_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        x_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reverse the right intake
        intake_wheel_right.setDirection(CRServo.Direction.REVERSE);


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



        //Ready up
        robotState.setDriveState(RobotState.DriveState.IDLE);
        robotState.setIntakeState(RobotState.IntakeState.IDLE);
        robotState.setLiftState(RobotState.LiftState.IDLE);
        robotState.setVisionState(RobotState.VisionState.DISABLED);

        //Ready for launch!!!!!
        robotState.setMainState(RobotState.MainState.IDLE);
        showTelemetry.startTelemetry();
        showTelemetry.updateTelem.updateTelemetry();


        waitForStart();



        robotState.setMainState(RobotState.MainState.ACTIVE);



        //HERE IS WHERE WE CAN PUT THE ORIGINAL OP MODE MECANUM CODE LOOP().
        //SOME THINGS CAN BE CHANGED DUE TO A LINEAR OP SETTING


        while(opModeIsActive()){

            showTelemetry.updateTelem.setStepNumbV(stepNumber);
            showTelemetry.updateTelem.updateTelemetry();



            int goalType = (int) goalLibrary[stepNumber][0]; //Setting goal each time

            //OLD SWITCH STATEMENT

            switch(goalType) {


                /* Position Change */
                case 0:
                    robotState.setDriveState(RobotState.DriveState.MOVING);
                    showTelemetry.updateTelem.updateTelemetry();

                    MovePosition();

                    robotState.setDriveState(RobotState.DriveState.IDLE);
                    showTelemetry.updateTelem.updateTelemetry();
                    break;


                /* Angle Change */
                case 1:
                    robotState.setDriveState(RobotState.DriveState.MOVING);
                    showTelemetry.updateTelem.updateTelemetry();

                    MoveAngle();

                    robotState.setDriveState(RobotState.DriveState.IDLE);
                    showTelemetry.updateTelem.updateTelemetry();
                    break;




                /* Vuforia.java :( ugh... */
                case 2:
                        robotState.setVisionState(RobotState.VisionState.SEARCHING);
                        showTelemetry.updateTelem.updateTelemetry();

                        stonePos = blockPosBlue.visionTest();
                        if(stonePos == 4) {
                            stonePos = 1;
                            robotState.setVisionState(RobotState.VisionState.ERROR);
                            showTelemetry.updateTelem.updateTelemetry();
                        }
                        robotState.setVisionState(RobotState.VisionState.FOUND);
                        showTelemetry.updateTelem.updateTelemetry();
                        if(goalLibrary[stepNumber][1] == 1){ //for blue side
                            robotState.setVisionState(RobotState.VisionState.DISABLED);
                            showTelemetry.updateTelem.updateTelemetry();
                            switch(stonePos) {
                                case 0: //bridge

                                    MoveOdomPosition(17.0d, 17.0d); /*change this */
                                    waitForDrive();

                                    //open claw, move arm up, etc

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    //close claw

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    goalLibrary[12][1] = 0;//change steps in library because we know where other Skystone is
                                    goalLibrary[12][2] = 0;

                                    break;
                                case 1: //center
                                    //open claw, move arm up, etc

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    //close claw

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    goalLibrary[13][0] = 1;
                                    goalLibrary[13][1] = 0; //change steps in library because we know where other Skystone is

                                    break;
                                case 2: //wall
                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    //get stone

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    goalLibrary[14][1] = 0; //change steps in library because we know where other Skystone is
                                    goalLibrary[14][2] = 0;

                                    break;

                            }

                        }

                        else{ //Red side
                            robotState.setVisionState(RobotState.VisionState.DISABLED);
                            showTelemetry.updateTelem.updateTelemetry();
                            switch (stonePos){
                                case 0:

                                    MoveOdomPosition(17.0d, 17.0d); /*change this */
                                    waitForDrive();

                                    //open claw, move arm up, etc

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    //close claw

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();


                                    goalLibrary[12][1] = 0;//change steps in library because we know where other Skystone is
                                    goalLibrary[12][2] = 0;
                                    break;
                                case 1:
                                    MoveOdomPosition(17.0d, 17.0d); /*change this */
                                    waitForDrive();

                                    //open claw, move arm up, etc

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    //close claw

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();


                                    goalLibrary[13][0] = 1;
                                    goalLibrary[13][1] = 0; //change steps in library because we know where other Skystone is
                                    break;
                                case 2:
                                    MoveOdomPosition(17.0d, 17.0d); /*change this */
                                    waitForDrive();

                                    //open claw, move arm up, etc

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    //close claw

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();

                                    MoveOdomPosition(17.0d, 17.0d);
                                    waitForDrive();


                                    goalLibrary[14][1] = 0; //change steps in library because we know where other Skystone is
                                    goalLibrary[14][2] = 0;
                                    break;

                            }
                        }
                    break;

                    //MAYBE INTAKE HERE:
                case 3:
                    intake();
                    break;


                //LIFT MOTOR UP:
                case 4:
                    robotState.setLiftState(RobotState.LiftState.MOVING_UP);
                    showTelemetry.updateTelem.updateTelemetry();

                    lift.setTargetPosition(1000);
                    lift.setPower(0.5);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(lift.isBusy()){
                        sleep(1);
                    }

                    robotState.setLiftState(RobotState.LiftState.IDLE);
                    showTelemetry.updateTelem.updateTelemetry();
                    break;

                //LIFT MOTOR DOWN:
                case 5:
                    break;


                //WAIT:
                case 6:
                    robotState.setMainState(RobotState.MainState.IDLE);
                    showTelemetry.updateTelem.updateTelemetry();

                    //Wait for indicated time
                    sleep((long) goalLibrary[stepNumber][1]);

                    robotState.setMainState(RobotState.MainState.ACTIVE);
                    showTelemetry.updateTelem.updateTelemetry();
                    break;
            }
            stepNumber++;



        }


    }
}
