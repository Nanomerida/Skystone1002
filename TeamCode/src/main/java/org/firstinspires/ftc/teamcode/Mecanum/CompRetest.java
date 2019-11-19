package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CRVuforia.VuforiaBlue;
import org.firstinspires.ftc.teamcode.Methods.GeneralMethods;

@Autonomous(name="Auton Test", group = "Autonomous")
public class CompRetest extends LinearOpMode {

    VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file
    GeneralMethods methods = new GeneralMethods();
    private ElapsedTime refreshTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    DcMotor left_front_drive   = null;
    DcMotor  left_back_drive  = null;
    DcMotor  right_front_drive = null;
    DcMotor  right_back_drive = null;
    DcMotor arm = null;
    CRServo claw = null;
    public WebcamName webcam = null;

    public BNO055IMU imu;
    private Orientation angles;


    private double degreesConversion(){
        while(refreshTimer.milliseconds() < 3){sleep(1);}
        refreshTimer.reset();
        double theta = AngleUnit.normalizeDegrees(this.angles.firstAngle);
        if(theta < 180) theta += 180;
        else theta -= 180;
        return theta;
    }


    @Override
    public void runOpMode(){


        left_front_drive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");

        arm = hardwareMap.get(DcMotor.class, "arm");

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        claw = hardwareMap.get(CRServo.class, "claw");

        blockPosBlue.blueInit(webcam);


        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);



        // MORE IMU STUFF

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        waitForStart();



        sleep(1000);



        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_front_drive.setPower(-0.3);
        left_back_drive.setPower(-0.3);
        right_front_drive.setPower(-0.3);
        right_back_drive.setPower(-0.3);



        sleep(660);



        int skystonePos = blockPosBlue.visionTest();



        switch(skystonePos){
            case 0:

                left_front_drive.setPower(-0.6);
                left_back_drive.setPower(0.9);
                right_front_drive.setPower(0.6);
                right_back_drive.setPower(-0.9);

                sleep(800);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                sleep(600);


                left_front_drive.setPower(-0.3);
                left_back_drive.setPower(-0.3);
                right_front_drive.setPower(-0.3);
                right_back_drive.setPower(-0.3);

                sleep(1140);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                claw.setPower(-0.3);
                sleep(3000);
                claw.setPower(-0.1);

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

                break;

            case 1:


                left_front_drive.setPower(-0.3);
                left_back_drive.setPower(-0.3);
                right_front_drive.setPower(-0.3);
                right_back_drive.setPower(-0.3);

                sleep(1140);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                claw.setPower(-0.3);
                sleep(3000);
                claw.setPower(-0.1);

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

                break;

            case 2:

                left_front_drive.setPower(0.6);
                left_back_drive.setPower(-0.9);
                right_front_drive.setPower(-0.6);
                right_back_drive.setPower(0.9);


                sleep(800);


                left_front_drive.setPower(-0.3);
                left_back_drive.setPower(-0.3);
                right_front_drive.setPower(-0.3);
                right_back_drive.setPower(-0.3);

                sleep(1140);

                left_front_drive.setPower(0);
                left_back_drive.setPower(0);
                right_front_drive.setPower(0);
                right_back_drive.setPower(0);

                claw.setPower(-0.3);
                sleep(3000);
                claw.setPower(-0.1);

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



        }


        left_front_drive.setPower(-0.6);
        left_back_drive.setPower(0.9);
        right_front_drive.setPower(0.6);
        right_back_drive.setPower(-0.9);

        sleep(1800);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        sleep(500);

        claw.setPower(0.3);
        sleep(2000);
        claw.setPower(0);

        sleep(500);

        left_front_drive.setPower(0.6);
        left_back_drive.setPower(-0.9);
        right_front_drive.setPower(-0.6);
        right_back_drive.setPower(0.9);

        sleep(900);



        idle();






    }
}
