package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;


import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.BulkDataManager;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Mecanum.Config.RobotDimensions;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;

import java.util.List;


public class CROdometry implements Subsystem {


    private RevBulkData bulkData;

    public BulkDataManager bulkDataManager;
    public Pose2d globalPos;

    public ExternalEncoder left_y_encoder;
    public ExternalEncoder right_y_encoder;
    public ExternalEncoder x_encoder;
    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;

    public FTCLibOdometry odometry;

    private double previousAngle;
    private double currentAngle;


    private static double trackWidth = 14.72071;
    private static final double DISTANCE_TO_TOP_RIGHT_X = 8.267717;
    private static final double DISTANCE_TO_TOP_RIGHT_Y = 8.724134;

    /**
     * The expansion hub with the odometers is needed. The passing of the opMode is necessary because
     * the program will send true as if it is at the position if the opMode ends while inside the class/
     */
    public CROdometry( ExpansionHubEx expansionHubEx, Pose2d globalPos, HardwareMap hardwareMap) {

        left_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "left_front_drive");
        left_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "left_back_drive");
        right_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "right_front_drive");
        right_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "right_back_drive");

        this.odometry = new FTCLibOdometry(globalPos, trackWidth);

        left_y_encoder = new ExternalEncoder((ExpansionHubMotor) hardwareMap.get(DcMotor.class, "left_y_encoder"), bulkData);
        right_y_encoder = new ExternalEncoder((ExpansionHubMotor) hardwareMap.get(DcMotor.class, "right_y_encoder"), bulkData);
        //We actually have both the encoder and the left lift motor in the same port.
        x_encoder = new ExternalEncoder((ExpansionHubMotor) hardwareMap.get(DcMotor.class, "lift_left"), bulkData);

        bulkDataManager = new BulkDataManager(expansionHubEx, bulkData);
        bulkDataManager.refreshBulkData();

        this.globalPos = globalPos;
    }

    @Override
    public void init(){

        //Set up the drive base
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reverse the opposite drive motors
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);

        //Hard reset all encoders
        left_y_encoder.hardReset();
        right_y_encoder.hardReset();
        x_encoder.hardReset();

        //Reverse right encoder
        right_y_encoder.reverseEncoder();

    }


    public boolean MoveOdomPosition(double GoalX, double GoalY, double theta) {

        boolean goalReachedPos;
        //Use odometry to move to a given position
        bulkDataManager.refreshBulkData();

        odometry.update(AngleUnit.RADIANS.toRadians(theta), x_encoder.getInches(), left_y_encoder.getInches(), right_y_encoder.getInches()); //update pos


        goalReachedPos = GoalCheckPos(globalPos.getPos().getX(), GoalX, globalPos.getPos().getY(), GoalY); //check if we are at position

        if (!goalReachedPos)
            setDrivePower(PositionChange(globalPos.getPos().getX(), GoalX, globalPos.getPos().getY(), GoalY)); //If we aren't, set power again
        else setDrivePower(new double[]{0, 0, 0, 0});

        syncEncoders();

        return goalReachedPos;


    }

    public boolean MoveAngle(double thetaG, double[] theta) {
        boolean goalReachedAngle;
        previousAngle = theta[0];
        currentAngle = theta[1];
        //Use odometry to rotate

        bulkDataManager.refreshBulkData();

        //NEED TO RECALL METHOD UNTIL DONE!!!!!!!

        odometry.update(AngleUnit.DEGREES.toRadians(currentAngle), (x_encoder.getInches() - ((AngleUnit.DEGREES.toRadians(currentAngle)
                        - AngleUnit.DEGREES.toRadians(previousAngle)) * 0.686417)),
                left_y_encoder.getInches(), right_y_encoder.getInches());

        goalReachedAngle = GoalCheckAngle(thetaG, currentAngle); //check if we are at angle.
        if (!goalReachedAngle) {
            if(thetaG > previousAngle) {
                setDrivePower(AngleChange(thetaG, currentAngle));
            }
            else {
                setDrivePower(negate(AngleChange(thetaG, currentAngle)));
            }
        }
        else setDrivePower(new double[]{0, 0, 0, 0});

        syncEncoders();

        return goalReachedAngle;

    }

    /**There is no way to account for turning odometry here, we are going to hav to accept it will be off */
    public void update(double theta){

        bulkDataManager.refreshBulkData();

        odometry.update(AngleUnit.RADIANS.toRadians(theta), x_encoder.getInches(), left_y_encoder.getInches(), right_y_encoder.getInches()); //update pos

        syncEncoders();

    }


    /** LINEAR MOVEMENT PROCEDURES */
    /*public double[] AbsolutePosition(double theta) {
        robotPosition.setHeading(theta);
        double[] PreviousPosition = robotPosition.getPosition();
        double[] CurrentPosition = new double[2];
        double[] XEncoderPosition = new double[2];
        double[] YEncoderPosition = new double[2];
        //double ConvRate = 0.006184246955207152778;
        double[] WeirdOrlandoMathsX = {sin(theta), cos(theta)};
        double[] WeirdOrlandoMathsY = {cos(theta), sin(theta)};
        //Get bulk data from hub
        bulkData = expansionHubEx.getBulkInputData();
        //get x encoder
        int XClicks = x_encoder.getCounts(bulkData);
        //Get the y encoders
        int YClicks = ((left_y_encoder.getCounts(bulkData) + right_y_encoder.getCounts(bulkData)) / 2);
        for (int i = 0; i < 2; i++) {
            XEncoderPosition[i] = ConvRate * XClicks * WeirdOrlandoMathsX[i];
            YEncoderPosition[i] = ConvRate * YClicks * WeirdOrlandoMathsY[i];
        }
        for (int k = 0; k < 2; k++) {
            CurrentPosition[k] = XEncoderPosition[k] + YEncoderPosition[k] + PreviousPosition[k];
        }
        syncEncoders();
        robotPosition.setPosition(CurrentPosition[0], CurrentPosition[1]);
        return CurrentPosition;
    } */

    private double[] PositionChange(double Xg, double Xa, double Yg, double Ya) {
        float[] MoveYBasePower = {0.55f, 1.0f, 0.5f, 1.0f};                             //Base Motor Power for Y movement
        float[] MoveXBasePower = {0.8f, -0.95f, -0.85f, 0.95f};                            //Base Motor Power for X movement
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

    private boolean GoalCheckPos(double Xa, double Xg, double Ya, double Yg) { /**/

        boolean reachedGoal;
        reachedGoal = (abs(Xg - Xa) < 1) && abs(Yg - Ya) < 1;
        return reachedGoal;
    }


    /**
     * Note: wheelDelta - angleChange * wheelOffset
     */
    /*public double[] TurningPosition(double theta, double previous) {
        robotPosition.setHeading(theta);
        double[] PreviousPosition = robotPosition.getPosition();
        double[] CurrentPosition = new double[2];
        double[] XEncoderPosition = new double[2];
        double[] YEncoderPosition = new double[2];
        //double ConvRate = 0.006184246955207152778; // Change this
        double[] WeirdOrlandoMathsX = {sin(theta), cos(theta)};
        double[] WeirdOrlandoMathsY = {cos(theta), sin(theta)};
        //Get bulk data from hub
        bulkData = expansionHubEx.getBulkInputData();
        //get x encoder
        int XClicks = (int) (x_encoder.getCounts(bulkData) - ((theta - previous) * 1));
        //Get the y encoders
        int YClicks = ((left_y_encoder.getCounts(bulkData) + right_y_encoder.getCounts(bulkData)) / 2);
        for (int i = 0; i < 2; i++) {
            XEncoderPosition[i] = ConvRate * XClicks * WeirdOrlandoMathsX[i];
            YEncoderPosition[i] = ConvRate * YClicks * WeirdOrlandoMathsY[i];
        }
        for (int k = 0; k < 2; k++) {
            CurrentPosition[k] = XEncoderPosition[k] + YEncoderPosition[k] + PreviousPosition[k];
        }
        syncEncoders();
        robotPosition.setPosition(CurrentPosition[0], CurrentPosition[1]);
        return CurrentPosition;
    } */

    private double[] AngleChange(double thetaG, double thetaA) { /**/
        float[] TurnBasePower = {0.65f, 1.0f, -0.65f, -1.0f};
        thetaA -= thetaG;
        double[] motorPower = new double[4];
        for (int i = 0; i < 4; i++) {
            motorPower[i] = thetaA * TurnBasePower[i];
        }
        return motorPower;
    }

    private boolean GoalCheckAngle(double thetaG, double thetaA) { /**/
        boolean reachedGoal = false;
        if (abs(thetaG - thetaA) < 1) {
            reachedGoal = true;
        }
        return reachedGoal;
    }

    private void setDrivePower(double[] powers) {
        left_front_drive.setPower(powers[0]);
        left_back_drive.setPower(powers[1]);
        right_front_drive.setPower(powers[2]);
        right_back_drive.setPower(powers[3]);
    }


    public void syncEncoders() {
        left_y_encoder.syncEncoders();
        right_y_encoder.syncEncoders();
        x_encoder.syncEncoders();
    }

     public static double[] negate(double[] pre){
        double[] post = new double[pre.length];
        for(int i = 0; i < pre.length; i++){
            post[i] = -pre[i];
        }
        return post;
    }

}
