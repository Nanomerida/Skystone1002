package org.firstinspires.ftc.teamcode.Methods;

import static java.lang.Math.abs;
import org.firstinspires.ftc.teamcode.Mecanum.MainAutonomousLinear;

import org.firstinspires.ftc.teamcode.Variables.Reference;

import java.util.HashMap;

public class GeneralMethods  implements MecanumMovement{

    public GeneralMethods(){

    }
    MainAutonomousLinear heading = new MainAutonomousLinear();
    Reference vars = new Reference();
    MecMoveProcedureStorage procedures = new MecMoveProcedureStorage();
    private HashMap<String, float[]> mecanum = procedures.getMecanum();

    // Position change method for mecanun
    public double[] PositionChange(double Xg, double Xa, double Yg, double Ya) { /**/
        float[] MoveYBasePower = mecanum.get("forward");                             //Base Motor Power for Y movement
        float[] MoveXBasePower = mecanum.get("strafeR");                            //Base Motor Power for X movement
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
        float[] TurnBasePower = mecanum.get("turnCC");
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
    public boolean GoalCheckAngle(double thetaG, double thetaA) { /**/
        boolean reachedGoal = false;
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
