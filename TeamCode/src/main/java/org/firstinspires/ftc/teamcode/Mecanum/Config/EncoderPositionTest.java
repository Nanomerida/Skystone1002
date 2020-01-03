package org.firstinspires.ftc.teamcode.Mecanum.Config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class EncoderPositionTest extends OpMode {


    public DcMotor left_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor right_back_drive = null;

    Telemetry.Item leftFront;
    Telemetry.Item leftBack;
    Telemetry.Item rightFront;
    Telemetry.Item rightBack;

    @Override
    public void init(){
        left_front_drive =  hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive =  hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive =  hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive =  hardwareMap.get(DcMotor.class, "right_back_drive");

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);

        leftFront = telemetry.addData("Left Front Position", () -> left_front_drive.getCurrentPosition());
        leftBack = telemetry.addData("Left Back Position", () -> left_back_drive.getCurrentPosition());
        rightFront = telemetry.addData("Right Front Position", () -> right_front_drive.getCurrentPosition());
        rightBack = telemetry.addData("Right Back Position", () -> right_back_drive.getCurrentPosition());

    }

    @Override
    public void loop(){

        telemetry.update();


    }
}
