package org.firstinspires.ftc.teamcode.OKIIIcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Outreach TeleOp", group="TeleOp")
public class Oktoberfest extends OpMode {

    public DcMotor left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;

    public DcMotor claw = null;

    private boolean slowModeOn = false;
    private boolean prevX = false;





    @Override
    public void init() {


        left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        claw = hardwareMap.get(DcMotor.class, "claw");


        //Reset ALL encoders
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        //double right = -gamepad1.right_stick_y;
        //double left = -gamepad1.left_stick_y;

        if(gamepad1.x && !prevX) slowModeOn = !slowModeOn;

        float speed = (slowModeOn) ? 1.0f : 0.5f;
/*
        // Set joystick values to motor values on robot
        left_front_drive.setPower(left * speed);
        left_back_drive.setPower(left * speed);
        right_front_drive.setPower(right * speed);
        right_back_drive.setPower(right * speed);

        */


        if(gamepad1.dpad_up){
            left_front_drive.setPower(1);
            left_back_drive.setPower(1);
            right_front_drive.setPower(1);
            right_back_drive.setPower(1);
        }
        else if(gamepad1.dpad_down){
            left_front_drive.setPower(-1);
            left_back_drive.setPower(-1);
            right_front_drive.setPower(-1);
            right_back_drive.setPower(-1);
        }
        else if(gamepad1.dpad_left){
            left_front_drive.setPower(1);
            left_back_drive.setPower(1);
            right_front_drive.setPower(0);
            right_back_drive.setPower(0);
        }
        else if(gamepad1.dpad_right){
            left_front_drive.setPower(0);
            left_back_drive.setPower(0);
            right_front_drive.setPower(1);
            right_back_drive.setPower(1);
        }
        else {
            left_front_drive.setPower(0);
            left_back_drive.setPower(0);
            right_front_drive.setPower(0);
            right_back_drive.setPower(0);
        }



        if(gamepad1.right_trigger>0.5){
            claw.setPower(0.5);
        } else {
            claw.setPower(0);
        }

        if(gamepad1.left_trigger>0.5){
            claw.setPower(-0.5);
        } else {
            claw.setPower(0);
        }


        // update prevX
        prevX = gamepad1.x;    }

    @Override
    public void stop() {
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);
        claw.setPower(0);


        /*
        claw_rotate.setPosition(0);
        claw.setPosition(0);
        */
    }
}