package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Current motor PIDs", group = "Testing")
@Disabled
public class TestingPID extends OpMode {


    private DcMotorEx left_front_drive;
    private DcMotorEx left_back_drive;
    private DcMotorEx right_front_drive;
    private DcMotorEx right_back_drive;


    @Override
    public void init(){
        left_front_drive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        left_back_drive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        right_front_drive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        right_back_drive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        left_front_drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){


        double leftPower =  gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;

        left_front_drive.setPower(leftPower);
        left_back_drive.setPower(leftPower);
        right_front_drive.setPower(rightPower);
        right_back_drive.setPower(rightPower);








            telemetry.addData(" Left Front Motor Current PID:", left_front_drive.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
            telemetry.addData(" Left Back Motor Current PID:", left_back_drive.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Right Front Motor Current PID:", right_front_drive.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Right Back Motor Current PID:", right_back_drive.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));

            telemetry.addData("Left Front Motor Current Velocity:", left_front_drive.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Left Back Motor Current Velocity:", left_back_drive.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Right Front Motor Current Velocity:", right_front_drive.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Right Back Motor Current Velocity:", right_back_drive.getVelocity(AngleUnit.DEGREES));

            telemetry.update();

    }

    @Override
    public void stop(){
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);
    }


}
