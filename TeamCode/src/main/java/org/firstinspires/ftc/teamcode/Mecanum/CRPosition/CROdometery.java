package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;


import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

    private OpMode opMode;

    public CROdometery(OpMode opMode, ExpansionHubEx expansionHubEx, ExpansionHubMotor left_y_encoder, ExpansionHubMotor right_y_encoder, ExpansionHubMotor x_encoder,
                       double[] start, double heading){

        this.opMode = opMode;
        this.expansionHubEx = expansionHubEx;
        this.left_y_encoder = new ExternalEncoder(left_y_encoder);
        this.right_y_encoder = new ExternalEncoder(right_y_encoder);
        this.x_encoder = new ExternalEncoder(x_encoder);
        this.robotPosition = new RobotPosition(start[0], start[1], heading, "Degrees");
        bulkData = this.expansionHubEx.getBulkInputData();
    }

    public double[] AbsolutePosition(double theta) {
        robotPosition.setHeading(theta);
        double[] PreviousPosition = robotPosition.getPosition();
        double[] CurrentPosition = new double[2];
        double[] XEncoderPosition = new double[2];
        double[] YEncoderPosition = new double[2];
        double ConvRate = (PI * 90) / 208076.8; /** Change this */
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

    public void syncEncoders(){
        left_y_encoder.syncEncoders(bulkData);
        right_y_encoder.syncEncoders(bulkData);
        x_encoder.syncEncoders(bulkData);
    }

}
