package org.firstinspires.ftc.teamcode.hardwareMaps;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;



public class BulkDataHub {

    ExpansionHubEx expansionHub;
    RevBulkData bulkData;
    ExpansionHubMotor left_y_encoder, right_y_encoder, x_encoder = null;
    
    
    BNO055IMU imu;
    private Orientation angles;
    private float heading;
    
    OpMode opMode;
    
    public BulkDataHub(OpMode aOpMode, String expansionHubName, DcMotor[] encoders, BNBNO055IMU revImu) {
      this.opMode = aOpMode;
      this.expansionHub = aOpMode.hardwareMap.get(ExpansionHubEx.class, expansionHubName);
      this.left_y_encoder = (ExpansionHubMotor) encoders[0];
      this.right_y_encoder = (ExpansionHubMotor) encoders[1];
      this.x_encoder = (ExpansionHubMotor) encoders[2];
      this.imu = revImu;
      
    }
    
    public void refreshData() {
      bulkData = expansionHub.getBulkInputData();
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      heading = angles.firstAngle;
    }
    
    public float getHeading(){
      return heading;
    }
    
    public double getYTicks(){
      double averageY = (bulkData.getMotorCurrentPosition(left_y_encoder) + bulkData.getMotorCurrentPosition(right_y_encoder)) / 2;
      return averageY;
    }
    
    public double getYTicks(){
      return bulkData.getMotorCurrentPosition(x_encoder);
    }
    
}
