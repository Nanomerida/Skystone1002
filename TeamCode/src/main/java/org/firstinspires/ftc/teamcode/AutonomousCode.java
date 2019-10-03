package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import static java.lang.Math.cos; //Ryan's Math Stuff
import static java.lang.Math.sin;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

@Autonomous //Assuming this is right
//@Disabled
public class AutonomousCode extends OpMode {

    CRGreenHardwareMap robot = new CRGreenHardwareMap(telemetry); //need to define the hardware map

    DcMotor left_front_drive;
    DcMotor left_back_drive;
    DcMotor right_front_drive;
    DcMotor right_back_drive;
    DcMotor main_arm;
    Servo claw_level;
    Servo claw;



    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;


    public double[] PositionChange(double Xg, double Xa, double Yg, double Ya) {
        double[] MoveYBasePower = {1.0000d, 1.0000d, 1.0000d, 1.0000d};                             //Base Motor Power for Y movement
        double[] MoveXBasePower = {1.0000d, -1.0000d, -1.0000d, 1.0000};                            //Base Motor Power for X movement
        Yg -= Ya;
        Xg -= Xa;
        Yg /= 144;
        Xg /= 144;
        for (int j = 0; j < 4; j++) {
            MoveYBasePower[j] *= Yg;
            MoveXBasePower[j] *= Xg;
        }
        double[] motorPower = new double[4];
        for (int k = 0; k < 4; k++) {
            motorPower[k] = MoveXBasePower[k] + MoveYBasePower[k];
        }
        return motorPower;
    }
    public double[] AngleChange(double thetaA, double thetaG) {
        double[] TurnBasePower = {1.0000d, -1.0000d, 1.0000d, -1.0000d};
        thetaA -= thetaG;
        double[] motorPower = new double[4];
        for(int i = 0; i < 4; i++) {
            motorPower[i] = thetaA * TurnBasePower[i];
        }
        return motorPower;
    }
    public double[] AbsolutePosition(double PrevX, double PrevY) {
        double[] PreviousPosition = {PrevX, PrevY};
        double[] CurrentPosition = new double[2];
        double[] XEncoderPosition = new double[2];
        double[] YEncoderPosition = new double[2];
        double ConvRate = (PI * 90) / 208076.8;
        double Heading = this.angles.firstAngle;
        double[] WeirdOrlandoMathsX = {sin(Heading), cos(Heading)};
        double[] WeirdOrlandoMathsY = {cos(Heading), sin(Heading)};
        double XClicks = {"encoders detected since previous iteration X"};
        double YClicks = {"encoders detected since previous iteration Y"};
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
    public boolean GoalCheckPos(double Xa, double Xg, double Ya, double Yg) {
        boolean reachedGoal = false;
        if(abs(Xg - Xa) < 0.1) {
            reachedGoal = true;
            if (abs(Yg - Ya) < 0.1) {
                reachedGoal = true;
            }
        }
        else{
            reachedGoal = false;
        }
        return reachedGoal;
    }

    public boolean GoalCheckAngle(double thetaG) {
        boolean reachedGoal = false;
        double thetaA = this.angles.firstAngle;
        double thetaDif = (thetaG - thetaA);
        if(abs(thetaDif) < 0.1) {
            reachedGoal = true;
        }
        return reachedGoal;
    }

    public void moveMotor(double[] powers) { //method to move.
        left_front_drive.setPower(powers[0]);
        left_back_drive.setPower(powers[1]);
        right_front_drive.setPower(powers[2]);
        right_back_drive.setPower(powers[3]);
        //right, Ima head out.
    }



    public static double[][] goalLibrary = {{0, 0, 0}, {1, 0, 0}};// {0, x, y}, {2, 0, 0}, {0, x, y}, {4, 0, 0}, {3, 0, 0}, {0, x, y}, {1, angle, 0}, {0, x, y}} //Blank steps for vuforia,{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, x, y}, {1, 0, 0}, continu....};

        //0 is a position change in format {0, x, y}
        //1 is an angle change in format {1, angle, 0}
        //2 is the vision test for skystone in format {2, 0, 0}
        //3 is an claw change in format {3, arm power, claw head servo}
        //4 is an arm change in format {4, open(0)/close(1) claw, 0}
        //others as needed

    Vuforia blockPos = new Vuforia(); //creates a Vuforia.java object from the Vuforia.java.java class.
    public static int stepNumber = 0;
    public static boolean newGoal = true; //variable if new goal is desired
    public static double[] previousPos = {0.00d, 0.00d}; //define starting position here. May change based on placement.
    /* Need goal check for angle, similar to position goal check. */




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        left_front_drive = hardwareMap.dcMotor.get("leftFrontMotor");
        left_back_drive = hardwareMap.dcMotor.get("leftBackMotor");
        right_front_drive = hardwareMap.dcMotor.get("rightFrontMotor");
        right_back_drive = hardwareMap.dcMotor.get("rightBackMotor");
        main_arm = hardwareMap.dcMotor.get("armMotor");
        claw_level = hardwareMap.servo.get("clawLevelServo");
        claw = hardwareMap.servo.get("clawServo");

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
        telemetry.addData("Glitches in MATRIX detected: 0", 4);
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double goalType = goalLibrary[stepNumber][0]; //Setting goal each time
        newGoal = false;

        /* Position Change */
        if(goalType == 0){
            double[] actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
            previousPos[0] = actualPos[0];
            previousPos[1] = actualPos[1]; // set previous position values for next position calculation.
            double[] motorPowerPos = PositionChange(actualPos[0], goalLibrary[stepNumber][1], actualPos[2], goalLibrary[stepNumber][2]);
            moveMotor(motorPowerPos);
            actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
            boolean goalReachedPos = GoalCheckPos(actualPos[0], goalLibrary [stepNumber][1], actualPos [1], goalLibrary [stepNumber][2]);
            if(goalReachedPos){
                newGoal = true;
            }
        }

        /* Angle Change */
        if(goalType == 1) {
            double[] motorPowerAngle = AngleChange(this.angles.firstAngle, goalLibrary[stepNumber][1]);
            moveMotor(motorPowerAngle);
            boolean goalReachedAngle = GoalCheckAngle(goalLibrary[stepNumber][1]); //need to make this
            if(goalReachedAngle){
                newGoal = true;
            }
        }
        /* Vuforia.java :( ugh... */
        if(goalType == 2) {
            int stonePos = blockPos.visionTest(); //outputs skystone position, in integer form.
            switch(stonePos) {
                case 0: //first position (left, towards skybridge)
                    go there;
                    grab thing;
                    go back;
                    goalLibrary [12][1] = x;//change steps in library because we know where other Skystone is
                    goalLibrary [12][2] = y;
                    break;
                case 1: //second position (center)
                    go there;
                    grab thing;
                    go back;
                    goalLibrary [13][0] = 1;
                    goalLibrary [13][1] = angle; //change steps in library because we know where other Skystone is
                    break;
                case 2: //third position (right, towards wall)
                    go there;
                    grab thing;
                    go back;
                    goalLibrary [14][1] = x; //change steps in library because we know where other Skystone is
                    goalLibrary [14][2] = y;
                    break;
            }
        }
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

