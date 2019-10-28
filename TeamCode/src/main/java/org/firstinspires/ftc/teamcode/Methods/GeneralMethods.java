package org.firstinspires.ftc.teamcode.Methods;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.OldCode.AutonomousCode;
import org.firstinspires.ftc.teamcode.Variables.Reference;

public class GeneralMethods {

    public GeneralMethods(){

    }
    AutonomousCode heading = new AutonomousCode();
    Reference vars = new Reference();

    // Position change method for mecanum
    public double[] PositionChange(double Xg, double Xa, double Yg, double Ya) { /**/
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

    //Angle change method for mecanum
    public double[] AngleChange(double thetaG) { /**/
        double thetaA = heading.degreesConversion();
        double[] TurnBasePower = {1.0000d, -1.0000d, 1.0000d, -1.0000d};
        thetaA -= thetaG;
        double[] motorPower = new double[4];
        for(int i = 0; i < 4; i++) {
            motorPower[i] = thetaA * TurnBasePower[i];
        }
        return motorPower;
    }

    //Goal check method for position
    public boolean GoalCheckPos(double Xa, double Xg, double Ya, double Yg) { /**/
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

    //Goal check method for angle
    public boolean GoalCheckAngle(double thetaG) { /**/
        boolean reachedGoal = false;
        double thetaA = heading.degreesConversion();
        double thetaDif = (thetaG - thetaA);
        if(abs(thetaDif) < 0.1) {
            reachedGoal = true;
        }
        return reachedGoal;
    }

    //Degrees to power conversion for the Servo
    public double degreeServoConv(double degrees){ /**/
        return degrees * vars.servoDegreesConst;
    }

    //

}
