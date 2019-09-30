package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math*;

@Autonomous(name="Autonomous", group="bruh") //Assuming this is right
//@Disabled
public class Autonomous extends OpMode {
    public static double[] PositionChange(double Xg, double Xa, double Yg, double Ya) {
        double MoveYBasePower[] = {1.0000d, 1.0000d, 1.0000d, 1.0000d};                             //Base Motor Power for Y movement
        double MoveXBasePower[] = {1.0000d, -1.0000d, -1.0000d, 1.0000};                            //Base Motor Power for X movement
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
    public static double[] AngleChange(double thetaA, double thetaG) {
        double TurnBasePower[] = {1.0000d, -1.0000d, 1.0000d, -1.0000d};
        thetaA -= thetaG;
        double[] motorPower = new double[4];
        for(int i = 0; i < 4; i++) {
            motorPower[i] = thetaA * TurnBasePower[i];
        }
        return motorPower;
    }
    public static double[] AbsolutePosition(double PrevX, double PrevY) {
        double PreviousPosition[] = {PrevX, PrevY};
        double CurrentPosition[] = new double[2];
        double XEncoderPosition[] = new double[2];
        double YEncoderPosition[] = new double[2];
        double ConvRate = Math.PI * 90 / 208076.8;
        double Heading = "current reading from gyro";
        double WeirdOrlandoMathsX[] = {sin(Heading), cos(Heading)};
        double WeirdOrlandoMathsY[] = {cos(Heading), sin(Heading)};
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
    public static boolean GoalCheckPos(double Xa, double Xg, double Ya, double Yg) {
        boolean reachedGoal = true
        if((Xg - Xa) < 0.1) {
            reachedGoal = true;
        }
        else if (((Yg - Ya) < 0.1) {
            reachedGoal = true;
        }
        else{
            reachedGoal = false;
        }
        return reachedGoal;
    }
    public double[][] goalLibrary = {{0, x, y},
            {1, angle, 0}, {0, x, y}, {2, 0, 0}, {0, x, y}, {4, 0, 0}, {3, 0, 0},
            {0, x, y}, {1, angle, 0}, {0, x, y}, etc...., //Blank steps for vuforia,
            {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
            {0, x, y}, {1, 0, 0}, continu....};

        //0 is a position change in format {0, x, y}
        //1 is an angle change in format {1, angle, 0}
        //2 is the vision test for skystone in format {2, 0, 0}
        //3 is an claw change in format {3, angle, orientation}
        //4 is an arm change in format {4, angle, height}
        //others as needed
    int stepNumber = 0;
    boolean newGoal = true; //variable if new goal is desired
    double previousPos[] = {0.00d, 0.00d}; //define starting position here. May chnage based on placement.




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Define methods to be used here. Also for goal library and other variables used in loop.
         * NOTE: Maybe put some more of these methods in a class?*/

        /* Method for Position change, outputs motor powers needed. */

        /* Method for angle change, outputs motor powers needed, */


        /* Method for goal check for position */

        /* Need goal check for angle, similar to position goal check. */


        /* Method for other things. */

        /* Goal Library */

        //Not a jagged array, so each internal array must be the same length

    }

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int goalType = goalLibrary [stepNumber ][0]; //Setting goal each time
        newGoal = true;

        /* Position Change */
        if(goalType == 0){
            double actualPos[] = AbsolutePosition(previousPos[0], previousPos[1]);
            previousPos[0] = actualPos[0];
            previousPos[1] = actualPos[1]; // set previous position values for next position calculation.
            double motorPowerPos[] = PositionChange(actualPos[0], goalLibrary[stepNumber][1], actualPos[2], goalLibrary[stepNumber][2]);
            robot.leftFrontMotor.setPower(motorPowerPos[0]);
            robot.leftBackMotor.setPower(motorPowerPos[1]);
            robot.rightFrontMotor.setPower(motorPowerPos[2]);
            robot.rightBackMotor.setPower(motorPowerPos[3]);
            actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
            boolean goalReachedPos = GoalCheckPos(actualPos[0], goalLibrary [stepNumber][1], actualPos [1], goalLibrary [stepNumber][2]);
            newGoal = (goalReachedPos == true) ? true : false;
        }

        /* Angle Change */
        if(goalType == 1) {
            double motorPowerAngle[] = AngleChange(goalLibrary [stepNumber][1]);
            robot.leftFrontMotor.setPower(motorPowerAngle[0]);
            robot.leftBackMotor.setPower(motorPowerAngle[1]);
            robot.rightFrontMotor.setPower(motorPowerAngle[2]);
            robot.rightBackMotor.setPower(motorPowerAngle[3]);
            boolean goalReachedAngle = GoalCheckAngle(goalLibrary[stepNumber][1]); //need to make this
            newGoal = (goalReachedAngle == true) ? true : false;
        }
        /* Vuforia :( ugh... */
        if(goalType == 2) {
            Vuforia blockPos = new Vuforia(); //creates a Vuforia object from the Vuforia.java class.
            int stonePos = blockPos.visionTest(); //outputs skystone position, in integer form.
            switch(stonePos) {
                case 0: //first position (left, towards skybridge)
                    go there;
                    grab thing;
                    go back;
                    goalLibrary [12][1] = x //change steps in library because we know where other Skystone is
                    goalLibrary [12][2] = y
                    break;
                case 1: //second position (center)
                    go there;
                    grab thing;
                    go back;
                    goalLibrary [13][0] = 1
                    goalLibrary [13][1] = angle //change steps in library because we know where other Skystone is
                    break;
                case 2: //third position (right, towards wall)
                    go there;
                    grab thing;
                    go back;
                    goalLibrary [14][1] = x //change steps in library because we know where other Skystone is
                    goalLibrary [14][2] = y
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

