package org.firstinspires.ftc.teamcode.Mecanum.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



public class TeleOpFieldCentric {

    private BNO055IMU imu;
    Orientation angles;




    public TeleOpFieldCentric(BNO055IMU imu){
        this.imu = imu;
    }


    double getHeadingRadians() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -angles.firstAngle;

        /*
        angles.firstAngle is the heading, measured COUNTER-CLOCKWISE, from the orientation the bot was in when the
        op mode was started (by default, straight upward). In the driveFieldRelative method, the X axis is the robot's
        starting forward direction and the Y axis is 90 degrees CLOCKWISE from the X axis. Likewise, in the
        driveMechanum method, the strafe direction (robot-right) is 90 degrees CLOCKWISE from the forward direction.
        It is these choices of coordinate systems that necessitate using the negative of angles.firstAngle.
         */

    }

    public double degreeFromRadians(double theta) {
        return theta * 360 / (2 * Math.PI);
    }

    public float[] driveFieldRelative(double x, double y, double rotate) {
        //Convert x, y to theta, r

        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y, x);

        //Get modified theta, r based off gyro heading
        double heading = getHeadingRadians();

        double modifiedTheta = theta - heading;

        //Convert theta and r back to a forward and strafe
        double forward = r * Math.cos(modifiedTheta);
        double strafe = r * Math.sin(modifiedTheta);

        return new float[] {(float)forward, (float)strafe, (float)rotate};
    }



}
