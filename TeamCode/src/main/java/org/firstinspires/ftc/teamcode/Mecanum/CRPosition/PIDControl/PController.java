package org.firstinspires.ftc.teamcode.Mecanum.CRPosition.PIDControl;

public interface PController {

    void setMax(double maxPower);

    void setMin(double minPower);

    void setkP(double kP);

    double getMax();

    double getMin();

    double getkP();

    double getOutput(double error);



}
