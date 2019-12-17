package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;


import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.abs;

import java.util.List;


public class CROdometry {


    private RobotPosition robotPosition;
    private RevBulkData bulkData;
    private ExpansionHubEx expansionHubEx;
    public Pose2d globalPos;

    private ExternalEncoder left_y_encoder;
    private ExternalEncoder right_y_encoder;
    private ExternalEncoder x_encoder;
    public DcMotor left_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor right_back_drive = null;

    private FTCLibOdometry odometry;

    private double previousAngle;
    private double currentAngle;
    private double  ConvRate = 0.006184246955207152778;

    private LinearOpMode opMode;

    /** All hardware must be initialized.
     * Encoders should go in List in the order of left y, right y, x.
     * Please remember to reverse the right y encoder.*/
    public CROdometry(LinearOpMode opMode, ExpansionHubEx expansionHubEx, List<ExpansionHubMotor> encoders,
                       double[] start, double heading, List<DcMotor> driveMotors) {

        this.opMode = opMode;
        this.odometry = new FTCLibOdometry(new Pose2d(start[0], start[1], heading), 18);
        this.expansionHubEx = expansionHubEx;
        this.left_y_encoder = new ExternalEncoder(encoders.get(0));
        this.right_y_encoder = new ExternalEncoder(encoders.get(1));
        this.x_encoder = new ExternalEncoder(encoders.get(2));
        this.robotPosition = new RobotPosition(start[0], start[1], heading, "Degrees");
        this.left_front_drive = driveMotors.get(0);
        this.left_back_drive = driveMotors.get(1);
        this.right_front_drive = driveMotors.get(2);
        this.right_back_drive = driveMotors.get(3);
        bulkData = this.expansionHubEx.getBulkInputData();
        globalPos = odometry.robotPos;
    }


    public boolean MoveOdomPosition(double GoalX, double GoalY, double theta) {
        boolean goalReachedPos;
        //Use odometry to move to a given position
        bulkData = expansionHubEx.getBulkInputData();

        odometry.update(theta, (x_encoder.getCounts(bulkData) * ConvRate), (left_y_encoder.getCounts(bulkData) * ConvRate), (right_y_encoder.getCounts(bulkData)
        * ConvRate));


        goalReachedPos = GoalCheckPos(globalPos.getPos().getX(), GoalX, globalPos.getPos().getY(), GoalY); //check if we are at position

        if (!goalReachedPos && opMode.opModeIsActive()) setDrivePower(PositionChange(globalPos.getPos().getX(), GoalX, globalPos.getPos().getY(), GoalY)); //If we aren't, set power again

        syncEncoders();

        return goalReachedPos;
    }

    public boolean MoveAngle(double thetaG, double[] theta) {
        boolean goalReachedAngle;
        previousAngle = theta[0];
        currentAngle = theta[1];
        //Use odometry to rotate

        //NEED TO RECALL METHOD UNTIL DONE!!!!!!!

        odometry.update(AngleUnit.DEGREES.toRadians(currentAngle), ((x_encoder.getCounts(bulkData) * ConvRate)-((AngleUnit.DEGREES.toRadians(currentAngle) - AngleUnit.DEGREES.toRadians(previousAngle)) * 1)),
                (left_y_encoder.getCounts(bulkData) * ConvRate), (right_y_encoder.getCounts(bulkData) * ConvRate));

        goalReachedAngle = GoalCheckAngle(thetaG, currentAngle); //check if we are at angle.
        if (!goalReachedAngle && opMode.opModeIsActive()) setDrivePower(AngleChange(thetaG, currentAngle));
        syncEncoders();
        return goalReachedAngle;

    }

    /** LINEAR MOVEMENT PROCEDURES */
    @Deprecated
    public double[] AbsolutePosition(double theta) {
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
    }

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
        if (abs(Xg - Xa) < 1) {
            reachedGoal = true;
            if (abs(Yg - Ya) < 1) {
                reachedGoal = true;
            }
            else {
                reachedGoal = false;
            }
        } else {
            reachedGoal = false;
        }
        return reachedGoal;
    }

    /** TURNING PROCEDURES */

    /**
     * Note: wheelDelta - angleChange * wheelOffset
     */
    @Deprecated
    public double[] TurningPosition(double theta, double previous) {
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
    }

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
        double thetaDif = (thetaG - thetaA);
        if (abs(thetaDif) < 1) {
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
        left_y_encoder.syncEncoders(bulkData);
        right_y_encoder.syncEncoders(bulkData);
        x_encoder.syncEncoders(bulkData);
    }

}
