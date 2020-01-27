package org.firstinspires.ftc.teamcode.Mecanum.LastResortAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Last Resort park", group = "Last Resort")
public class LastResortAutoDriveForward extends LinearOpMode {

    DcMotor lf = null;
    DcMotor lb = null;
    DcMotor rf = null;
    DcMotor rb = null;

    @Override
    public void runOpMode(){

        lf = hardwareMap.dcMotor.get("left_front_drive");
        lb = hardwareMap.dcMotor.get("left_back_drive");
        rf = hardwareMap.dcMotor.get("right_front_drive");
        rb = hardwareMap.dcMotor.get("right_back_drive");


        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        lf.setPower(0.5);
        lb.setPower(0.5);
        rf.setPower(0.5);
        rb.setPower(0.5);


        sleep(300);

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        idle();




    }
}