package org.firstinspires.ftc.teamcode.Mecanum.LastResortAutons;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CRVuforia.Vuforia;
import org.firstinspires.ftc.teamcode.Methods.GeneralMethods;
import org.firstinspires.ftc.teamcode.hardware.DriveBaseVectors;

@Autonomous(name="Last Resort BlueLoadingZone", group = "Last Resort")
public class LastResortAuton extends LinearOpMode {

    Vuforia blockPosBlue = new Vuforia(); //creates an instance of the vuforia blue side file
    GeneralMethods methods = new GeneralMethods();
    private ElapsedTime refreshTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    DcMotor left_front_drive = null;
    DcMotor left_back_drive = null;
    DcMotor right_front_drive = null;
    DcMotor right_back_drive = null;
    Servo claw = null;
    public WebcamName webcam = null;

    private BNO055IMU imu;
    private Orientation angles;


    private static final double TURNING_ANGLE_P = 2;
    private static final double HOLDING_ANGLE_P = 2.5;
    private static final double TURNING_ANGLE_THRESHOLD = 1;


    private void stopDrive(){
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);
    }


    /*So distance between the current angle and target angle is normalized to the range of -180 to 180, and then that value is passed to a p controller. */

    private void turnTo(double targetAngle, double speed){

        double error;
        double output;
        error = getError(targetAngle);
        output = getPOutput(error, TURNING_ANGLE_P);

        if(opModeIsActive()) {

            while (opModeIsActive() && (error >= TURNING_ANGLE_THRESHOLD)) {
                error = getError(targetAngle);
                output = getPOutput(error, TURNING_ANGLE_P);

                left_front_drive.setPower(DriveBaseVectors.turnCW[0] * output * speed);
                left_back_drive.setPower(DriveBaseVectors.turnCW[1] * output * speed);
                right_front_drive.setPower(DriveBaseVectors.turnCW[2] * output * speed);
                right_back_drive.setPower(DriveBaseVectors.turnCW[3] * output * speed);
            }
        }

    }

    /**
     * Controls power to stay at a designated heading (must be a 90 degree increment) and
     * also will stop if the opMode ends or the time runs out.
     *
     * Need to set the power before calling this
     *
     *
     *
     * Angle options:
     *
     * Straight Forward = 0
     * Strafe Left = negative 90
     * Strafe Right = positive 90
     * Backwards (may be very unstable, use cautiously) = somewhere from -175 to 175
     *
     * @param speed the overall speed of the movement
     * @param angle The angle to hold
     * @param timeInMillis The time to run for
     * @param relativeLeftMotors Which two motors control the left side of the robot
     * @param relativeRightMotors Which two motors control the right side of the robot
     */
    private void holdAngle(double speed, double timeInMillis, double angle,
                                  DcMotor[] relativeLeftMotors, DcMotor[] relativeRightMotors){

        if(opModeIsActive()) {
            // keep looping while we are still active, and BOTH motors are running.

            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            while (opModeIsActive() && (timer.milliseconds() <= timeInMillis)) {

                // adjust relative speed based on heading error.
                double error = getError(angle);
                double steer = getPOutput(error, HOLDING_ANGLE_P);
                double leftSpeed;
                double rightSpeed;


                leftSpeed = speed - steer;
                rightSpeed = speed + steer;


                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                for (DcMotor motor : relativeLeftMotors) {
                    motor.setPower(motor.getPower() * leftSpeed);
                }

                for (DcMotor motor : relativeRightMotors) {
                    motor.setPower(motor.getPower() * rightSpeed);
                }

            }
        }

        stopDrive();
    }
    /**
     * Returns the relative error between the current angle and the target angle
     * Target angle must be within the -180/+180 range.
     * NOTE: DO NOT ASK FOR 180 or -180 degrees everything breaks
     * @param targetAngle Target Angle within -180/+180 range
     * @return The error normalized to -180/+180
     */
    private double getError(double targetAngle) {

        // calculate error in -179 to +180 range  (
        double robotError = targetAngle - degreesConversion();
        return AngleUnit.normalizeDegrees(robotError);
    }

    private double getPOutput(double error, double PCoefficient){
        return Range.clip(error * PCoefficient, -1, 1);
    }

    private double degreesConversion(){
        double theta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        theta = AngleUnit.normalizeDegrees(theta + 90);
        return theta;
    }


    /**
     * Nelitha here is a sample usage of the above controls
     *
     * <br>
     *     <br>
     *
     *
     * Say I want to strafe directly left from my current heading for 900 milliseconds
     *
     * Set the default power:
     *
     * <br>
     * {@code
     * left_front_drive.setPower(0.6);
     * left_back_drive.setPower(-0.9);
     * right_front_drive.setPower(-0.6);
     * right_back_drive.setPower(0.9);
     * }
     *
     * <br>
     *     <br>
     * Then hand over control to the controller with a speed of 0.9, a time of 900, an
     * angle of 90 degrees, and the back two motors as the "left" and the front two as the "right".
     * <br>
     *     <br>
     *
     * {@code holdAngle(0.9, 900, 90, new DcMotor[] {left_back_drive, right_back_drive},
     * new DcMotor[] {left_front_drive, right_front_drive}); }
     *
     *
     * <br>
     *     <br>
     *
     * For turning, no previous power setting is needed, just do
     *
     * {@code turnTo(targetAngle, speed) }
     *
     *
     *
     *
     */
    @Override
    public void runOpMode() {



        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        claw = hardwareMap.get(Servo.class, "claw");

        blockPosBlue.blueInit(webcam, this);


        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);


        // MORE IMU STUFF

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu 1".
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


/*
to strafe (right):
        left_front_drive.setPower(-0.55);
        left_back_drive.setPower(1);
        right_front_drive.setPower(0.43);
        right_back_drive.setPower(-1);

*/

        waitForStart();


        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





//---------------------------------------------------------------------------------------------------------------------------
        /**This picks up a random stone and drops it off. Puts it on foundation, then parks*/
/*
        left_front_drive.setPower(0.3);
        left_back_drive.setPower(0.3);
        right_front_drive.setPower(0.3);
        right_back_drive.setPower(0.3);

        sleep(1800);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);


        claw.setPower(-0.3);
        sleep(3000);
        claw.setPower(-0.1);

        sleep(500);

        left_front_drive.setPower(-0.3);
        left_back_drive.setPower(-0.3);
        right_front_drive.setPower(-0.3);
        right_back_drive.setPower(-0.3);

        sleep(700);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        sleep(500);

        left_front_drive.setPower(1);
        left_back_drive.setPower(-1);
        right_front_drive.setPower(-1);
        right_back_drive.setPower(1);

        sleep(1400);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        sleep(500);

        left_front_drive.setPower(0.3);
        left_back_drive.setPower(0.3);
        right_front_drive.setPower(0.3);
        right_back_drive.setPower(0.3);

        sleep(250);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        sleep(500);


        left_front_drive.setPower(-1);
        left_back_drive.setPower(-1);
        right_front_drive.setPower(1);
        right_back_drive.setPower(1);

        sleep(200);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        sleep(500);

        arm.setPower(-0.69420);
        sleep(2000);
        claw.setPower(0);

        sleep(500);

        claw.setPower(0.3);
        sleep(2000);
        claw.setPower(0);

        sleep(500);

        arm.setPower(0.69420);
        sleep(2000);
        claw.setPower(0);

        sleep(500);

        left_front_drive.setPower(-1);
        left_back_drive.setPower(-1);
        right_front_drive.setPower(1);
        right_back_drive.setPower(1);

        sleep(200);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        sleep(500);

        left_front_drive.setPower(-0.55);
        left_back_drive.setPower(1);
        right_front_drive.setPower(0.65);
        right_back_drive.setPower(-1);

        sleep(1000);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);



        idle();
*/

// ---------------------------------------------------------------------------------------------------------------------------
/**This code finds the skystone, scores it, and then parks*/


        left_front_drive.setPower(0.3);
        left_back_drive.setPower(0.3);
        right_front_drive.setPower(0.3);
        right_back_drive.setPower(0.3);

        sleep(750);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);


        while(opModeIsActive() && new ElapsedTime().milliseconds() >= 5){
            telemetry.update();
            blockPosBlue.updateVision();
        }
        blockPosBlue.stopVision();

        Vuforia.SkystonePosition pose = blockPosBlue.getResults();

        switch(pose) {
            case LEFT://left
                left_front_drive.setPower(0.6);
                left_back_drive.setPower(-0.9);
                right_front_drive.setPower(-0.6);
                right_back_drive.setPower(0.9);

                sleep(210);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(0.3);
                left_back_drive.setPower(0.3);
                right_front_drive.setPower(0.3);
                right_back_drive.setPower(0.3);

                sleep(1000);


                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);


                claw.setPosition(0.2);
                sleep(3000);
                claw.setPosition(.7);

                sleep(500);

                left_front_drive.setPower(-0.3);
                left_back_drive.setPower(-0.3);
                right_front_drive.setPower(-0.3);
                right_back_drive.setPower(-0.3);

                sleep(600);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(1);
                left_back_drive.setPower(1);
                right_front_drive.setPower(-1);
                right_back_drive.setPower(-1);

                sleep(200);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(-1);
                right_back_drive.setPower(1);

                sleep(1100);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                claw.setPosition(0.2);
                sleep(3000);
                claw.setPosition(.7);

                sleep(500);

                left_front_drive.setPower(-1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(1);
                right_back_drive.setPower(1);

                sleep(300);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(-0.55);
                left_back_drive.setPower(1);
                right_front_drive.setPower(0.65);
                right_back_drive.setPower(-1);

                sleep(600);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);
                break;

            case RIGHT://middle
                left_front_drive.setPower(0.3);
                left_back_drive.setPower(0.3);
                right_front_drive.setPower(0.3);
                right_back_drive.setPower(0.3);

                sleep(1000);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);


                claw.setPosition(0.2);
                sleep(3000);
                claw.setPosition(.7);

                sleep(500);

                left_front_drive.setPower(-0.3);
                left_back_drive.setPower(-0.3);
                right_front_drive.setPower(-0.3);
                right_back_drive.setPower(-0.3);

                sleep(475);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(-1);
                right_back_drive.setPower(1);

                sleep(1200);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                claw.setPosition(0.2);
                sleep(3000);
                claw.setPosition(.7);

                sleep(500);

                left_front_drive.setPower(-1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(1);
                right_back_drive.setPower(1);

                sleep(300);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(-0.55);
                left_back_drive.setPower(1);
                right_front_drive.setPower(0.65);
                right_back_drive.setPower(-1);

                sleep(600);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                break;

            case UNSEEN://right
                left_front_drive.setPower(-0.6);
                left_back_drive.setPower(0.9);
                right_front_drive.setPower(0.6);
                right_back_drive.setPower(-0.9);

                sleep(140);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(0.3);
                left_back_drive.setPower(0.3);
                right_front_drive.setPower(0.3);
                right_back_drive.setPower(0.3);

                sleep(1000);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);


                claw.setPosition(0.2);
                sleep(3000);
                claw.setPosition(.7);

                sleep(500);

                left_front_drive.setPower(-0.3);
                left_back_drive.setPower(-0.3);
                right_front_drive.setPower(-0.3);
                right_back_drive.setPower(-0.3);

                sleep(475);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(-1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(1);
                right_back_drive.setPower(1);

                sleep(200);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(-1);
                right_back_drive.setPower(1);

                sleep(1300);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                claw.setPosition(0.2);
                sleep(3000);
                claw.setPosition(.7);

                sleep(500);

                left_front_drive.setPower(-1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(1);
                right_back_drive.setPower(1);

                sleep(300);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(-0.55);
                left_back_drive.setPower(1);
                right_front_drive.setPower(0.65);
                right_back_drive.setPower(-1);

                sleep(700);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);
                break;
        }
//--------------------------------------------------------------------------------------------------------------------------------------------------------
/**This is just to test something*//*
        left_front_drive.setPower(0.3);
        left_back_drive.setPower(0.3);
        right_front_drive.setPower(0.3);
        right_back_drive.setPower(0.3);

        sleep(750);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);


        int skystonePos = blockPosBlue.visionTest();

        switch(skystonePos) {
            case 0://left
                left_front_drive.setPower(0.6);
                sleep(500);
          /*      left_back_drive.setPower(-0.9);
                right_front_drive.setPower(-0.6);
                right_back_drive.setPower(0.9);

                sleep(185);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(0.3);
                left_back_drive.setPower(0.3);
                right_front_drive.setPower(0.3);
                right_back_drive.setPower(0.3);

                sleep(1000);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(-0.3);
                left_back_drive.setPower(-0.3);
                right_front_drive.setPower(-0.3);
                right_back_drive.setPower(-0.3);

                sleep(600);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(1);
                left_back_drive.setPower(1);
                right_front_drive.setPower(-1);
                right_back_drive.setPower(-1);

                sleep(200);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(-1);
                right_back_drive.setPower(1);

                sleep(1400);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(0.3);
                left_back_drive.setPower(0.3);
                right_front_drive.setPower(0.3);
                right_back_drive.setPower(0.3);

                sleep(250);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);


                left_front_drive.setPower(-1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(1);
                right_back_drive.setPower(1);

                sleep(200);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                arm.setPower(-0.69420);
                sleep(2000);

                arm.setPower(0.69420);
                sleep(2000);


                left_front_drive.setPower(-1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(1);
                right_back_drive.setPower(1);

                sleep(200);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(-0.55);
                left_back_drive.setPower(1);
                right_front_drive.setPower(0.65);
                right_back_drive.setPower(-1);

                sleep(1000);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);*/
                /*break;
            case 1://middle
                left_front_drive.setPower(0.3);
                left_back_drive.setPower(0.3);
                right_front_drive.setPower(0.3);
                right_back_drive.setPower(0.3);

                sleep(1000);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(-0.3);
                left_back_drive.setPower(-0.3);
                right_front_drive.setPower(-0.3);
                right_back_drive.setPower(-0.3);

                sleep(475);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(-1);
                right_back_drive.setPower(1);

                sleep(1400);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(0.3);
                left_back_drive.setPower(0.3);
                right_front_drive.setPower(0.3);
                right_back_drive.setPower(0.3);

                sleep(250);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);


                left_front_drive.setPower(-1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(1);
                right_back_drive.setPower(1);

                sleep(200);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                arm.setPower(-0.69420);
                sleep(2000);

                arm.setPower(0.69420);
                sleep(2000);


                left_front_drive.setPower(-1);
                left_back_drive.setPower(-1);
                right_front_drive.setPower(1);
                right_back_drive.setPower(1);

                sleep(200);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(500);

                left_front_drive.setPower(-0.55);
                left_back_drive.setPower(1);
                right_front_drive.setPower(0.65);
                right_back_drive.setPower(-1);

                sleep(1000);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                break;
            case 2://right
                right_front_drive.setPower(1);
                sleep(500);
                break;
        }*/
    }
}