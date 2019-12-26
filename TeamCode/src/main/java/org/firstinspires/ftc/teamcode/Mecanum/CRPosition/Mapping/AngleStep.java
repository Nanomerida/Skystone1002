package org.firstinspires.ftc.teamcode.Mecanum.CRPosition.Mapping;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

public class AngleStep implements Step {


    double theta;
    long delay;

    public AngleStep(double theta, long delay){
        this.theta = theta;
        this.delay = delay;
    }

    public double getTheta(){
        return theta;
    }

    public double getDelay() {
        return delay;
    }

    @Override
    public double[] getAsArray(){
        return new double[] {theta, (double) delay};
    }
}
