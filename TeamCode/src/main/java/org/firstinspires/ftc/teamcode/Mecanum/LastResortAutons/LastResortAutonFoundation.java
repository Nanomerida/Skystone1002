package org.firstinspires.ftc.teamcode.Mecanum.LastResortAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Methods.GeneralMethods;

@Autonomous(name="Last Resort BlueFoundationZone", group = "Last Resort")
public class LastResortAutonFoundation extends LinearOpMode {

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

        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        left_front_drive.setPower(0.3);
        left_back_drive.setPower(0.3);
        right_front_drive.setPower(0.3);
        right_back_drive.setPower(0.3);

        sleep(1800);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);


        arm.setPower(0.7);
        sleep(3000);
        arm.setPower(0);

        arm.setPower(-0.7);
        sleep(3000);
        arm.setPower(0);

        sleep(500);


        left_front_drive.setPower(-0.3);
        left_back_drive.setPower(-0.3);
        right_front_drive.setPower(-0.3);
        right_back_drive.setPower(-0.3);

        sleep(1800);

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        left_front_drive.setPower(-0.6);
        left_back_drive.setPower(0.9);
        right_front_drive.setPower(0.6);
        right_back_drive.setPower(-0.9);
        sleep(1000);
    }
}
