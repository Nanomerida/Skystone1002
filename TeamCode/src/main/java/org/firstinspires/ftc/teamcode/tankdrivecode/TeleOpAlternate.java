package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;
import org.firstinspires.ftc.teamcode.Methods.*;
import org.firstinspires.ftc.teamcode.Variables.*;


@TeleOp(name="ACTUAL TELEOP", group="TeleOp")
public class TeleOpAlternate extends OpMode {


    public DcMotor left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;
    
    public Servo claw = null;

    private boolean slowModeOn = false;
    private boolean prevX = false;





    @Override
    public void init() {


        left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        
        claw = hardwareMap.get(Servo.class, "claw");


        //Reset ALL encoders
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double right = -gamepad1.right_stick_y;
        double left = -gamepad1.left_stick_y;

        if(gamepad1.x && !prevX) slowModeOn = !slowModeOn;

        float speed = (slowModeOn) ? 0.5f : 1.0f;

        // Set joystick values to motor values on robot
        left_front_drive.setPower(left * speed);
        left_back_drive.setPower(left * speed);
        right_front_drive.setPower(right * speed);
        right_back_drive.setPower(right * speed);
        
        if(gamepad2.a){
            claw.setPosition(0);
        }
        
        if(gamepad2.b){
            claw.setPosition(0.5);
        }
/*
        //DOES THIS WORK?
        main_arm.setPower(gamepad2.right_stick_y);

        if(gamepad2.a) {
            slide.setPower(-0.2);
        } else {
            slide.setPower(0.0);
        }

        if(gamepad2.y){
            slide.setPower(0.2);
        } else {
            slide.setPower(0.0);
        }

        if(gamepad2.dpad_up) claw_level.setPosition(claw_level.getPosition() + 0.03);

        if(gamepad2.dpad_down) claw_level.setPosition(claw_level.getPosition() - 0.03);


        if(gamepad2.dpad_left)  claw_rotate.setPosition(0); //think this is horizontal to robot

        if(gamepad2.dpad_right) claw_rotate.setPosition(0.5); //and i think this is vertical to the robot

        if(gamepad2.left_trigger>0.9) claw.setPosition(0.5); //130 degrees from 180 (closed)

        if(gamepad2.right_trigger>0.9) claw.setPosition(0); // Full open*/

        // update prevX
        prevX = gamepad1.x;    }

    @Override
    public void stop() {
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

    
        /*
        claw_rotate.setPosition(0);
        claw.setPosition(0);
        */
    }
}
