package org.firstinspires.ftc.teamcode.Mecanum.CRPosition.PIDControl;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mecanum.CRPosition.Vector2d;

import java.lang.Math;


public class DistanceController implements PController {



    private double maxPower;
    private double minPower;
    private PIDCoefficients coefficients;


    public DistanceController(PIDCoefficients coefficients){
        this.coefficients = coefficients;
    }


    public double[] applyControl(Vector2d target_pos, Vector2d current_pos, double[] power){



        double error = distanceFormula(target_pos, current_pos);


        /*There can't be negative distance,
         * so Math.abs() is not needed.
         */
        if (error <= 0.5){
            return new double[] {0,0,0,0};
        }
        double output = getOutput(error);

        double[] correction = new double[4];
        /*
        Apply the output to all the elements, and normalize them
         */
        for(int i = 0; i < 4; i++){
            correction[i] = Range.clip(power[i] * output, -1.0, 1.0);
        }
        return correction;
    }

    @Override
    public void setMax(double maxPower){
        this.maxPower = maxPower;
    }
    @Override
    public void setMin(double minPower){
        this.minPower = minPower;
    }

    @Override
    public void setCoefficients(PIDCoefficients coefficients){
        this.coefficients = coefficients;
    }

    @Override
    public double getMax(){
        return maxPower;
    }

    @Override
    public double getMin(){
        return minPower;
    }

    @Override
    public PIDCoefficients getCoefficients(){
        return coefficients;
    }

    @Override
    public double getOutput(double error){
        return error * coefficients.kP;
    }

    private static double distanceFormula(Vector2d p1, Vector2d p2){

        double ac = Math.abs(p2.getY() - p1.getY());
        double cb = Math.abs(p2.getX() - p1.getX());

        return Math.hypot(ac, cb);
    }

}
