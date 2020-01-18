package org.firstinspires.ftc.teamcode.Mecanum.LastResortAutons;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CRVuforia.EasyOpenCv.SubsystemVision;
import org.firstinspires.ftc.teamcode.CRVuforia.Vuforia;
import org.firstinspires.ftc.teamcode.Methods.GeneralMethods;
import org.firstinspires.ftc.teamcode.hardware.DriveBaseVectors;

@Autonomous(name="Last Resort BlueLoadingZone", group = "Last Resort")
public class LastResortAuton extends LinearOpMode {

    class TimeBasedMover {

        private DcMotor left_front_drive = null;
        private DcMotor left_back_drive = null;
        private DcMotor right_front_drive = null;
        private DcMotor right_back_drive = null;

        TimeBasedMover(DcMotor[] drive){
            left_front_drive = drive[0];
            left_back_drive = drive[1];
            right_front_drive = drive[2];
            right_back_drive = drive[3];
        }

        public void goForward(double speed){
            left_front_drive.setPower(DriveBaseVectors.forward[0] * speed);
            left_back_drive.setPower(DriveBaseVectors.forward[1] * speed);
            right_front_drive.setPower(DriveBaseVectors.forward[2] * speed);
            right_back_drive.setPower(DriveBaseVectors.forward[3] * speed);
        }

        public void goBackward(double speed){
            left_front_drive.setPower(DriveBaseVectors.backward[0] * speed);
            left_back_drive.setPower(DriveBaseVectors.backward[1] * speed);
            right_front_drive.setPower(DriveBaseVectors.backward[2] * speed);
            right_back_drive.setPower(DriveBaseVectors.backward[3] * speed);
        }

        public void strafeL(double speed){
            left_front_drive.setPower(-DriveBaseVectors.strafeR[0] * speed);
            left_back_drive.setPower(-DriveBaseVectors.strafeR[1] * speed);
            right_front_drive.setPower(-DriveBaseVectors.strafeR[2] * speed);
            right_back_drive.setPower(-DriveBaseVectors.strafeR[3] * speed);
        }

        public void strafeR(double speed){
            left_front_drive.setPower(DriveBaseVectors.strafeR[0] * speed);
            left_back_drive.setPower(DriveBaseVectors.strafeR[1] * speed);
            right_front_drive.setPower(DriveBaseVectors.strafeR[2] * speed);
            right_back_drive.setPower(DriveBaseVectors.strafeR[3] * speed);
        }

        public void stopDrive(){
            left_front_drive.setPower(0);
            left_back_drive.setPower(0);
            right_front_drive.setPower(0);
            right_back_drive.setPower(0);
        }

    }

    private ElapsedTime refreshTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    DcMotor left_front_drive = null;
    DcMotor left_back_drive = null;
    DcMotor right_front_drive = null;
    DcMotor right_back_drive = null;
    Servo claw = null;
    Servo arm = null;

    private BNO055IMU imu;
    private Orientation angles;


    private static final double TURNING_ANGLE_P = 2;
    private static final double TURNING_ANGLE_THRESHOLD = 1;

    private static int skystonePosition;

    private TimeBasedMover drive;



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


            left_front_drive.setPower(0);
            left_back_drive.setPower(0);
            right_front_drive.setPower(0);
            right_back_drive.setPower(0);
        }

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

        //This must be in the init because it needs the hardware map
        SubsystemVision vision = new SubsystemVision(hardwareMap, this);

        vision.initHardware();



        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");

        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");


        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);

        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize a simple drive base
        drive = new TimeBasedMover(new DcMotor[] {left_front_drive, left_back_drive, right_front_drive, right_back_drive});


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

        telemetry.log().add("Moving arm to init pos");
        telemetry.update();
        arm.setPosition(0);

        sleep(300);

        telemetry.log().add("Moving claw to init pos");
        telemetry.update();
        claw.setPosition(0.4);


        telemetry.log().add("Vision Started!");
        vision.startVision();
        //Run vision in init
        while (!isStarted()){

            vision.streamLoop();
            telemetry.update();

        }

        //Clear all the vision telemetry
        telemetry.clearAll();


        //Retrieve the last known results
        skystonePosition = vision.pipeline.getDetectedSkystonePosition();

        telemetry.log().add("Skystone Position is " + skystonePosition);










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


        drive.goForward(0.75);

        sleep(750);

        drive.stopDrive();

        sleep(100);




        switch(skystonePosition) {
            case 0://left

                //Go up.
                drive.goForward(0.5);

                sleep(1000);


                //Close claw
                claw.setPosition(0);
                sleep(700);

                //Raise arm slightly to reduce the drag of the stone
                arm.setPosition(0.1);

                //Back up
                drive.goBackward(0.5);

                sleep(600);

                drive.stopDrive();


                sleep(100);

                //Turn to face bridge
                turnTo(-90, 0.7);

                sleep(200);

                //Go under bridge
                drive.goForward(0.8);

                sleep(1400);

                //Stop and put down arm
                drive.stopDrive();
                arm.setPosition(0);

                sleep(200);

                //release stone
                claw.setPosition(0.4);

                sleep(500);

                //Go backward
                drive.goBackward(0.5);

                sleep(300);

                //Ending positions
                claw.setPosition(0);

                //Stop and park under bridge
                drive.stopDrive();

                break;

            case 1://middle

                //Align
                drive.strafeR(0.3);
                sleep(100);
                drive.stopDrive();
                sleep(100);

                //Move up
                drive.goForward(0.5);

                sleep(700);

                drive.stopDrive();


                //Grab stone
                claw.setPosition(0);

                sleep(500);

                //Raise arm slightly to reduce the drag of the stone
                arm.setPosition(0.1);

                //Back up
                drive.goBackward(0.5);

                sleep(600);

                drive.stopDrive();

                sleep(200);

                //Turn toward bridge
                turnTo(-90, 0.7);

                sleep(500);

                //Go under bridge
                drive.goForward(0.8);

                sleep(1500);

                //Stop and put down arm
                drive.stopDrive();
                arm.setPosition(0);

                sleep(300);

                //Release stone
                claw.setPosition(0.4);

                sleep(500);
                //Go backward
                drive.goBackward(0.5);

                sleep(300);

                //Ending positions
                claw.setPosition(0);

                //Stop and park under bridge
                drive.stopDrive();

                break;

            case 2://right
                drive.strafeR(0.8);

                sleep(140);

                drive.stopDrive();

                sleep(500);

                drive.goForward(0.5);

                sleep(1000);

                drive.stopDrive();


                claw.setPosition(0);

                sleep(500);
                arm.setPosition(0.1);

                drive.goBackward(0.5);

                sleep(475);

                drive.stopDrive();

                sleep(500);

                turnTo(-90, 0.7);

                sleep(200);

                drive.goForward(0.8);

                sleep(1300);

                drive.stopDrive();
                arm.setPosition(0);

                sleep(500);

                claw.setPosition(0.4);

                sleep(500);

                drive.goBackward(0.5);

                sleep(200);

                drive.stopDrive();

                sleep(500);

                claw.setPosition(0);

                sleep(700);
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