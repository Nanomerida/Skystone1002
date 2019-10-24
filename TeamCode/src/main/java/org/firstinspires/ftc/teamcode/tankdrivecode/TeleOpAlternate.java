package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;
import org.firstinspires.ftc.teamcode.Methods.*;
import org.firstinspires.ftc.teamcode.Variables.*;


@TeleOp(name="TeleOpAlternate", group="TeleOp")
public class TeleOpAlternate extends OpMode {

    HardwareMapMain robot = new HardwareMapMain();
    GeneralMethods methods = new GeneralMethods();
    Reference ref = new Reference();

    public DcMotor left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;
    public DcMotor  main_arm     = null;
    public DcMotor  slide = null;
    public Servo claw_level    = null;
    public Servo    claw   = null;
    public Servo    claw_rotate = null;

    private static final double START_POSITION_CLAW       =  0.0 ; //starting pose of main claw servo
    private static final double START_POSITION_CLAW_LEVELER = 0.0; //starting pose of the claw leveler
    private static final double START_POSITION_CLAW_ROTATER = 0.5; 

    HardwareMap hwMap           =  null;




    @Override
    public void init() {


        left_front_drive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hwMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hwMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hwMap.get(DcMotor.class, "rightBackDrive");

        //Arm
        slide = hwMap.get(DcMotor.class, "slide_motor");
        main_arm    = hwMap.get(DcMotor.class, "main_arm");
        claw_level = hwMap.get(Servo.class, "claw_leveler");
        claw = hwMap.get(Servo.class, "claw");
        claw_rotate = hwMap.get(Servo.class, "claw_rotate");

        //Reset ALL encoders
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        main_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        main_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);
        
        main_arm.setPower(0);
        slide.setPower(0)
        
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        
        claw_level.setPosition(START_POSITION_CLAW_LEVELER);
        claw.setPosition(START_POSITION_CLAW);
        claw_rotate.setPosition(START_POSITION_CLAW_ROTATER);
    }

    @Override
    public void loop() {
        double right = -gamepad1.right_stick_y;
        double left = -gamepad1.left_stick_y;

        // Set joystick values to motor values on robot
        left_front_drive.setPower(left);
        left_back_drive.setPower(left);
        right_front_drive.setPower(right);
        right_back_drive.setPower(right);

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

        if(gamepad2.dpad_left)  claw_rotate.setPosition(0); //think this is horizontal to robot

        if(gamepad2.dpad_right) claw_rotate.setPosition(.5); //and i think this is vertical to the robot

        if(gamepad2.left_trigger) claw.setPosition(0.277); //130 degrees from 180 (closed)

        if(gamepad2.right_trigger) claw.setPosition(1); // Full open 
    }

    @Override
    public void stop() {
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

        main_arm.setPower(0);
        slide.setPower(0);
        /*
        claw_rotate.setPosition(0);
        claw.setPosition(0);
        */
    }
}
