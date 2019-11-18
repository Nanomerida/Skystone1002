package org.firstinspires.ftc.teamcode.hardwareMaps;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;


public class ImuUtil {



    private float heading;

    private BNO055IMU imu;
    private Orientation angles;
    
    public ImuUtil(BNO055IMU imu) {
      this.imu = imu;
    }
    
    public float getHeading(){
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      heading = angles.firstAngle;
      return heading;
    }
    
    
    }
    
    
      
