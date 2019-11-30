package org.firstinspires.ftc.teamcode.Mecanum.LastResortAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

@Autonomous(name="Last Resort BlueLoadingZone", group = "Last Resort")
public class LastResortAuton extends LinearOpMode {

    VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file
    GeneralMethods methods = new GeneralMethods();
    private ElapsedTime refreshTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    DcMotor left_front_drive = null;
    DcMotor left_back_drive = null;
    DcMotor right_front_drive = null;
    DcMotor right_back_drive = null;
    DcMotor arm = null;
    CRServo claw = null;
    public WebcamName webcam = null;

    //public BNO055IMU imu;
    //private Orientation angles;

    @Override
    public void runOpMode() {


        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");

        arm = hardwareMap.get(DcMotor.class, "arm");
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        claw = hardwareMap.get(CRServo.class, "claw");

        blockPosBlue.blueInit(webcam);


        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);


/*
to strafe (right):
        left_front_drive.setPower(-0.55);
        left_back_drive.setPower(1);
        right_front_drive.setPower(0.43);
        right_back_drive.setPower(-1);

*/

        waitForStart();


        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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


        int skystonePos = blockPosBlue.visionTest();

        switch(skystonePos){
            case 0://left
                left_front_drive.setPower(0.6);
                left_back_drive.setPower(-0.9);
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


                claw.setPower(-0.3);
                sleep(3000);
                claw.setPower(-0.1);

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

                claw.setPower(0.3);
                sleep(2000);
                claw.setPower(0);

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


                claw.setPower(-0.3);
                sleep(3000);
                claw.setPower(-0.1);

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

                claw.setPower(0.3);
                sleep(2000);
                claw.setPower(0);

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

            case 2://right
                left_front_drive.setPower(-0.6);
                left_back_drive.setPower(0.9);
                right_front_drive.setPower(0.6);
                right_back_drive.setPower(-0.9);

                sleep(160);

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


                claw.setPower(-0.3);
                sleep(3000);
                claw.setPower(-0.1);

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

                claw.setPower(0.3);
                sleep(2000);
                claw.setPower(0);

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