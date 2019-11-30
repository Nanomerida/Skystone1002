package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;


import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

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

/*    public CROdometery(OpMode opMode, ExpansionHubEx expansionHubEx, ExpansionHubMotor left_y_encoder, ExpansionHubMotor right_y_encoder, ExpansionHubMotor x_encoder,
                       double[] start, double heading, ArrayList<DcMotor> driveMotors){

        this.opMode = opMode;
        this.expansionHubEx = expansionHubEx;
        this.left_y_encoder = new ExternalEncoder(left_y_encoder);
        this.right_y_encoder = new ExternalEncoder(right_y_encoder);
        this.x_encoder = new ExternalEncoder(x_encoder);
        this.robotPosition = new RobotPosition(start[0], start[1], heading, "Degrees");
        this.left_front_drive = driveMotors.get(0);
        this.left_back_drive = driveMotors.get(1);
        this.right_front_drive = driveMotors.get(2);
        this.right_back_drive = driveMotors.get(3);
        bulkData = this.expansionHubEx.getBulkInputData();
    }
    
    
     public void MoveOdomPosition(double GoalX, double GoalY){
        boolean goalreachedPos;
        //Use odometry to move to a given position
        do {
            double[] previousPos = robotPosition.getPosition();
            double[] actualPos = Movement.Moving.AbsolutePosition(previousPos[0], previousPos[1]);
            goalReachedPos = Movement.Moving.GoalCheckPos(actualPos[0], GoalX, actualPos[1], GoalY); //check if we are at position
            //if (!goalReachedPos) moveDrivebyPower(Movement.Moving.PositionChange(actualPos[0], GoalX, actualPos[2], GoalY));
            previousPos[0] = actualPos[0];
            previousPos[1] = actualPos[1];
        }
        while(!goalReachedPos);
        stopDrive();
    }
    
    public class Movement {
        public static class Moving {
            
            public static double[] AbsolutePosition(double theta) {
                robotPosition.setHeading(theta);
                double[] PreviousPosition = robotPosition.getPosition();
                double[] CurrentPosition = new double[2];
                double[] XEncoderPosition = new double[2];
                double[] YEncoderPosition = new double[2];
                double ConvRate = (PI * 90) / 208076.8; // Change this
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
            }*/

       /*     public static double[] PositionChange(double Xg, double Xa, double Yg, double Ya) {
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
            
            public static boolean GoalCheckPos(double Xa, double Xg, double Ya, double Yg) { /**/
       /*
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
        public static class Turning {
            
            public static double[] AngleChange(double thetaG, double thetaA) { /**/
           /*     float[] TurnBasePower = {0.65f, 1.0f, -0.65f, -1.0f};
                thetaA -= thetaG;
                double[] motorPower = new double[4];
                for(int i = 0; i < 4; i++) {
                    motorPower[i] = thetaA * TurnBasePower[i];
                }
                return motorPower;
            }
            public static boolean GoalCheckAngle(double thetaG, double thetaA) { /**/
           /*     boolean reachedGoal = false;
                double thetaDif = (thetaG - thetaA);
                if(abs(thetaDif) < 1) {
                    reachedGoal = true;
                }
                return reachedGoal;
            }
        }
    }*/

    
    
    
    public void syncEncoders(){
        left_y_encoder.syncEncoders(bulkData);
        right_y_encoder.syncEncoders(bulkData);
        x_encoder.syncEncoders(bulkData);
    }

}
