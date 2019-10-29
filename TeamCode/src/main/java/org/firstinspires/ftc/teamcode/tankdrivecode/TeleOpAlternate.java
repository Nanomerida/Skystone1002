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


@TeleOp(name="YES, ACTUAL TELEOP", group="TeleOp")
public class TeleOpAlternate extends OpMode {


    public DcMotor left_drive   = null;
    public DcMotor  right_drive = null;
    
    //public DcMotor arm = null;
    //public Servo claw = null;

    private boolean slowModeOn = false;
    private boolean prevX = false;





    @Override
    public void init() {


        left_drive  = hardwareMap.get(DcMotor.class, "leftDrive");
        right_drive = hardwareMap.get(DcMotor.class, "rightDrive");
        //arm = hardwareMap.get(DcMotor.class, "arm");


        //Reset ALL encoders
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
        left_drive.setPower(left * speed);
        right_drive.setPower(right * speed);
/*
        //moves the arm
        if(gamepad2.right_trigger>0.5){
            arm.setPower(0.5);
        } else {
            arm.setPower(0);
        }
        
        if(gamepad2.left_trigger>0.5){
            arm.setPower(-0.5);
        } else {
            arm.setPower(0);
        }
        //moves the claw
        if(gamepad2.a) {
            claw.setPosition(-0.5);
        } else {
            claw.setPosition(0.0);
        }

        if(gamepad2.y) {
            claw.setPosition(0.5);
        } else {
            claw.setPosition(0.0);
        }*/


        // update prevX
        prevX = gamepad1.x;
    }

    @Override
    public void stop() {
        left_drive.setPower(0);
        right_drive.setPower(0);
        //arm.setPower(0);
    }
}
