package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;


import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;


public class Movement {
  
  ExternalEncoder left_y_encoder;
  ExternalEncoder right_y_encoder;
  ExternalEncoder x_encoder;
  RevBulkData bulkData;
  
  public double[] PositionChange(double Xg, double Xa, double Yg, double Ya) { /**/
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
  
  
  public double[] AngleChange(double thetaG, double thetaA) { /**/
        float[] TurnBasePower = {0.65f, 1.0f, -0.65f, -1.0f};
        thetaA -= thetaG;
        double[] motorPower = new double[4];
        for(int i = 0; i < 4; i++) {
            motorPower[i] = thetaA * TurnBasePower[i];
        }
        return motorPower;
    }
  public Movement(ExternalEncoder ly, ExternalEncoder ry, ExternalEnocoder x) {
    this.left_y_encoder = ly;
    this.right_y_encoder = ry;
    this.x_encoder = x;
  }
}
