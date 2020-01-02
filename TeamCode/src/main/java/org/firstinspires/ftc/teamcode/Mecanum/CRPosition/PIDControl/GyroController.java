package org.firstinspires.ftc.teamcode.Mecanum.CRPosition.PIDControl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static org.firstinspires.ftc.teamcode.hardware.DriveBaseVectors.turnCW;

public class GyroController implements PController {

    private double maxPower;
    private double minPower;
    private double kP;


    public GyroController(double kP){
        this.kP = kP;
    }


    public double[] applyControl(double targetAngle, double current_angle){

        double error = AngleUnit.normalizeDegrees(current_angle - targetAngle);
        if (Math.abs(error) <= 0.1){
            return new double[] {0,0,0,0};
        }
        double output = getOutput(error);

        double[] correction = new double[4];
        for(int i = 0; i < 4; i++){
            correction[i] = Range.clip(turnCW[i] * output, -1.0, 1.0);
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
    public void setkP(double kP){
        this.kP = kP;
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
    public double getkP(){
        return kP;
    }

    @Override
    public double getOutput(double error){
        return error * kP;
    }
}
