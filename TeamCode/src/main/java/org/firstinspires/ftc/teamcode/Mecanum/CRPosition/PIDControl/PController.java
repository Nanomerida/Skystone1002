package org.firstinspires.ftc.teamcode.Mecanum.CRPosition.PIDControl;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

public interface PController {

    void setMax(double maxPower);

    void setMin(double minPower);

    void setCoefficients(PIDCoefficients coefficients);

    double getMax();

    double getMin();

    PIDCoefficients getCoefficients();

    double getOutput(double error);



}
