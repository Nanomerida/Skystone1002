package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;
import org.firstinspires.ftc.teamcode.Variables.*;
import org.firstinspires.ftc.teamcode.Methods.*;

import static java.lang.Math.cos; //Ryan's Math Stuff
import static java.lang.Math.sin;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

@Autonomous //Assuming this is right
//@Disabled
public class AutonomousCode extends OpMode {

    HardwareMapMain robot = new HardwareMapMain(); //need to define the hardware map
    VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file
    VuforiaRed blockPosRed = new VuforiaRed(); //creates an instance of the vuforia red side file
    GoalLibraries library = new GoalLibraries();
    Reference constants = new Reference();
    GeneralMethods methods = new GeneralMethods();




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

    public void moveDrive(double[] powers) { //method to move.
        robot.left_front_drive.setPower(powers[0]);
        robot.left_back_drive.setPower(powers[1]);
        robot.right_front_drive.setPower(powers[2]);
        robot.right_back_drive.setPower(powers[3]);
        //right, Ima head out.
    }

    public double armClawPower(double armPower){ //write orlando's math here for ar power.
        telemetry.update();
        return armPower;
    }

    public void armMove(double armPower, double clawPower){ //convert to 1-0
        clawPower = methods.degreeServoConv(clawPower);
        robot.main_arm.setPower(armPower);
        robot.claw_level.setPosition(clawPower);
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
    public static final double clawClosed = 45.0d;
    public static final double clawOpen = 90.0d;
    public static final String driveIdle = "IDLE";
    public static final String driveMoving = "MOVING";
    public static final String driveTurning = "TURNING";
    HardwareMap hardwareMap;
    /* Need goal check for angle, similar to position goal check. */




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //HARDWARE MAP

        robot.init(hardwareMap);

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

        double goalType = goalLibrary[stepNumber][0]; //Setting goal each time
        newGoal = false;

        /* Position Change */
        if(goalType == 0){
            double[] actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
            previousPos[0] = actualPos[0];
            previousPos[1] = actualPos[1]; // set previous position values for next position calculation.
            double[] motorPowerPos = methods.PositionChange(actualPos[0], goalLibrary[stepNumber][1], actualPos[2], goalLibrary[stepNumber][2]); //move
            driveStatus.setValue(driveMoving);
            telemetry.update();
            moveDrive(motorPowerPos);
            driveStatus.setValue(driveIdle);
            telemetry.update();
            actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
            boolean goalReachedPos = methods.GoalCheckPos(actualPos[0], goalLibrary [stepNumber][1], actualPos [1], goalLibrary [stepNumber][2]); //check if we are at position
            if(goalReachedPos){
                newGoal = true;
            }
        }

        /* Angle Change */
        if(goalType == 1) {
            double[] motorPowerAngle = methods.AngleChange( goalLibrary[stepNumber][1]);
            driveStatus.setValue(driveTurning);
            telemetry.update();
            moveDrive(motorPowerAngle); //move
            driveStatus.setValue(driveIdle);
            telemetry.update();
            boolean goalReachedAngle = methods.GoalCheckAngle(goalLibrary[stepNumber][1]); //check if we are at angle.
            if(goalReachedAngle){
                newGoal = true;
            }
        }
        /* Vuforia.java :( ugh... */
        if(goalType == 2) {
            visionStatus.setValue("ENABLED; SEARCHING");
            telemetry.update();
            int stonePos = blockPosBlue.visionTest(); //outputs skystone position, in integer form.
            switch(stonePos) {
                case 0: //first position (left, towards skybridge)
                    visionStatus.setValue("SKYSTONE: TOWARDS BRIDGE");
                    telemetry.update();
                    //go there;
                    //grab thing;
                    //go back;
                    goalLibrary [12][1] = 0;//change steps in library because we know where other Skystone is
                    goalLibrary [12][2] = 0;
                    break;
                case 1: //second position (center)
                    visionStatus.setValue("SKYSTONE: CENTER");
                    telemetry.update();
                    //go there;
                    //grab thing;
                    //go back;
                    goalLibrary [13][0] = 1;
                    goalLibrary [13][1] = 0; //change steps in library because we know where other Skystone is
                    break;
                case 2: //third position (right, towards wall)
                    visionStatus.setValue("SKYSTONE: TOWARDS WALL");
                    telemetry.update();
                    //go there;
                    //grab thing;
                    //go back;
                    goalLibrary [14][1] = 0; //change steps in library because we know where other Skystone is
                    goalLibrary [14][2] = 0;
                    break;
            }
            visionStatus.setValue("DISABLED");
            telemetry.update();
            newGoal = true;
        }
        /* Arm Change */
        if(goalType == 3) {
            double clawLevelPower = armClawPower(goalLibrary[stepNumber][1]);
            armStatus.setValue("MOVING");
            clawLevelStatus.setValue("MOVING");
            telemetry.update();
            armMove(goalLibrary[stepNumber][1], clawLevelPower);
            armStatus.setValue("IDLE");
            clawLevelStatus.setValue("IDLE");
            telemetry.update();


        }
        if(goalType == 4) { //claw: open is 0, closed is 1
            if(goalLibrary[stepNumber][1] == 0){ //open claw
                clawStatus.setValue("OPENING");
                telemetry.update();
                robot.claw.setPosition(clawOpen);
            }
            else { //close claw
                clawStatus.setValue("CLOSING");
                telemetry.update();
                robot.claw.setPosition(clawClosed);
            }
            clawStatus.setValue("IDLE");
            telemetry.update();
        }
        if(newGoal){
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

