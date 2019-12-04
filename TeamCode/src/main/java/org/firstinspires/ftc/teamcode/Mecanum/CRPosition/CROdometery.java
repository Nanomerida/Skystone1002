package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;


import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.abs;



public class CROdometery {


    private RobotPosition robotPosition;
    private RevBulkData bulkData;
    private ExpansionHubEx expansionHubEx;

    private ExternalEncoder left_y_encoder;
    private ExternalEncoder right_y_encoder;
    private ExternalEncoder x_encoder;
    public DcMotor left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;

    private OpMode opMode;
    public Moving move;
    public Turning turn;
    
    private int previousAngle;
    public int currentAngle;

    public CROdometery(OpMode opMode, ExpansionHubEx expansionHubEx, ExpansionHubMotor left_y_encoder, ExpansionHubMotor right_y_encoder, ExpansionHubMotor x_encoder,
                       double[] start, double heading, DcMotor[] driveMotors){

        this.opMode = opMode;
        this.expansionHubEx = expansionHubEx;
        this.left_y_encoder = new ExternalEncoder(left_y_encoder);
        this.right_y_encoder = new ExternalEncoder(right_y_encoder);
        this.x_encoder = new ExternalEncoder(x_encoder);
        this.robotPosition = new RobotPosition(start[0], start[1], heading, "Degrees");
        this.left_front_drive = driveMotors[0];
        this.left_back_drive = driveMotors[1];
        this.right_front_drive = driveMotors[2];
        this.right_back_drive = driveMotors[3];
        bulkData = this.expansionHubEx.getBulkInputData();
        move = new Moving();
        turn = new Turning();
    }
    
    
     public boolean MoveOdomPosition(double GoalX, double GoalY, double theta){
        boolean goalreachedPos;
        //Use odometry to move to a given position
        do {
            double[] previousPos = robotPosition.getPosition();
            double[] actualPos = move.AbsolutePosition(theta);
            goalreachedPos = move.GoalCheckPos(actualPos[0], GoalX, actualPos[1], GoalY); //check if we are at position
            if (!goalreachedPos) setDrivePower(move.PositionChange(actualPos[0], GoalX, actualPos[2], GoalY));
            previousPos[0] = actualPos[0];
            previousPos[1] = actualPos[1];
        }
        while(!goalreachedPos);
        setDrivePower(new double[] {0,0,0,0});
        return goalReachedPos;
    }

    public boolean MoveAngle(double thetaG, double thetaA){
        boolean goalReachedAngle;
        previousAngle = thetaA;
        currentAngle = thetaA;
        //Use odometry to rotate
        do {
            //NEED TO RECALL METHOD UNTIL DONE!!!!!!!
            turn.TurningPosition(AngleUnit.DEGREES.toRadians(currentAngle), AngleUnit.DEGREES.toRadians(previousAngle));
            goalReachedAngle = turn.GoalCheckAngle(thetaG, thetaA); //check if we are at angle.
            if (!goalReachedAngle) {
                setDrivePower(turn.AngleChange(thetaG, thetaA));
            }
        }
        while(!goalReachedAngle);
        setDrivePower(new double[] {0,0,0,0});
        return goalRechedAngle;
    }
    
     public class Moving {

            public double[] AbsolutePosition(double theta) {
                robotPosition.setHeading(theta);
                double[] PreviousPosition = robotPosition.getPosition();
                double[] CurrentPosition = new double[2];
                double[] XEncoderPosition = new double[2];
                double[] YEncoderPosition = new double[2];
                double ConvRate = 0.006184246955207152778; // Change this
                double[] WeirdOrlandoMathsX = {sin(theta), cos(theta)};
                double[] WeirdOrlandoMathsY = {cos(theta), sin(theta)};
                //Get bulk data from hub
                bulkData = expansionHubEx.getBulkInputData();
                //get x encoder
                int XClicks = x_encoder.getCounts(bulkData);
                //Get the y encoders
                int YClicks = ((left_y_encoder.getCounts(bulkData) + right_y_encoder.getCounts(bulkData)) / 2);
                for(int i = 0; i < 2; i++) {
                    XEncoderPosition[i] = ConvRate * XClicks * WeirdOrlandoMathsX[i];
                    YEncoderPosition[i] = ConvRate * YClicks * WeirdOrlandoMathsY[i];
                }
                for(int k = 0; k < 2; k++) {
                    CurrentPosition[k] = XEncoderPosition[k] + YEncoderPosition[k] + PreviousPosition[k];
                }
                syncEncoders();
                robotPosition.setPosition(CurrentPosition[0], CurrentPosition[1]);
                return CurrentPosition;
            }

            public double[] PositionChange(double Xg, double Xa, double Yg, double Ya) {
                float[] MoveYBasePower = {0.55f, 1.0f, 0.5f, 1.0f};                             //Base Motor Power for Y movement
                float[] MoveXBasePower = {0.4f, -0.95f, -0.45f, 0.95f};                            //Base Motor Power for X movement
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

            public boolean GoalCheckPos(double Xa, double Xg, double Ya, double Yg) { /**/

                boolean reachedGoal = false;
                if(abs(Xg - Xa) < 1) {
                    reachedGoal = true;
                    if (abs(Yg - Ya) < 1) {
                        reachedGoal = true;
                    }
                }
                else {
                    reachedGoal = false;
                }
                return reachedGoal;
            }

        }
        public class Turning {
            /** Note: wheelDelta - angleChange * wheelOffset */
            public double[] TurningPositon(double theta, double previous){
                    robotPosition.setHeading(theta);
                    double[] PreviousPosition = robotPosition.getPosition();
                    double[] CurrentPosition = new double[2];
                    double[] XEncoderPosition = new double[2];
                    double[] YEncoderPosition = new double[2];
                    double ConvRate = 0.006184246955207152778; // Change this
                    double[] WeirdOrlandoMathsX = {sin(theta), cos(theta)};
                    double[] WeirdOrlandoMathsY = {cos(theta), sin(theta)};
                    //Get bulk data from hub
                    bulkData = expansionHubEx.getBulkInputData();
                    //get x encoder
                    int XClicks =  (int)(x_encoder.getCounts(bulkData) - ((theta - previous) * 1));
                    //Get the y encoders
                    int YClicks = ((left_y_encoder.getCounts(bulkData) + right_y_encoder.getCounts(bulkData)) / 2);
                    for(int i = 0; i < 2; i++) {
                        XEncoderPosition[i] = ConvRate * XClicks * WeirdOrlandoMathsX[i];
                        YEncoderPosition[i] = ConvRate * YClicks * WeirdOrlandoMathsY[i];
                    }
                    for(int k = 0; k < 2; k++) {
                        CurrentPosition[k] = XEncoderPosition[k] + YEncoderPosition[k] + PreviousPosition[k];
                    }
                    syncEncoders();
                    robotPosition.setPosition(CurrentPosition[0], CurrentPosition[1]);
                    return CurrentPosition;
            }

            public double[] AngleChange(double thetaG, double thetaA) { /**/
                float[] TurnBasePower = {0.65f, 1.0f, -0.65f, -1.0f};
                thetaA -= thetaG;
                double[] motorPower = new double[4];
                for(int i = 0; i < 4; i++) {
                    motorPower[i] = thetaA * TurnBasePower[i];
                }
                return motorPower;
            }
            public boolean GoalCheckAngle(double thetaG, double thetaA) { /**/
                boolean reachedGoal = false;
                double thetaDif = (thetaG - thetaA);
                if (abs(thetaDif) < 1) {
                    reachedGoal = true;
                }
                return reachedGoal;
            }
    }

    private void setDrivePower(double[] powers) {
        left_front_drive.setPower(powers[0]);
        left_back_drive.setPower(powers[1]);
        right_front_drive.setPower(powers[2]);
        right_back_drive.setPower(powers[3]);
    }
    
    
    public void syncEncoders(){
        left_y_encoder.syncEncoders(bulkData);
        right_y_encoder.syncEncoders(bulkData);
        x_encoder.syncEncoders(bulkData);
    }

}
