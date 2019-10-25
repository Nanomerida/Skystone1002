package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;
import org.firstinspires.ftc.teamcode.Methods.*;
import org.firstinspires.ftc.teamcode.Variables.*;


@TeleOp(name="TeleOpAlternate", group="TeleOp")
public class TeleOpAlternate extends OpMode {

    HardwareMapMain robot = new HardwareMapMain();
/*    HardwareMapMain robot = new HardwareMapMain();
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
    public Servo    claw_rotate = null;*/

    private static final double START_POSITION_CLAW       =  0.0 ; //starting pose of main claw servo
    private static final double START_POSITION_CLAW_LEVELER = 0.0; //starting pose of the claw leveler
    private static final double START_POSITION_CLAW_ROTATER = 0.5;
    private boolean slowModeOn = false;
    private boolean prevX = false;

    HardwareMap hwMap           =  null;




    @Override
    public void init() {
        robot.init(hardwareMap);
/*
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
        slide.setPower(0);
        */
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
        robot.left_front_drive.setPower(left * speed);
        robot.left_back_drive.setPower(left * speed);
        robot.right_front_drive.setPower(right * speed);
        robot.right_back_drive.setPower(right * speed);

        //DOES THIS WORK?
        robot.main_arm.setPower(gamepad2.right_stick_y);

        if(gamepad2.a) {
            robot.slide.setPower(-0.2);
        } else {
            robot.slide.setPower(0.0);
        }

        if(gamepad2.y){
            robot.slide.setPower(0.2);
        } else {
            robot.slide.setPower(0.0);
        }

        if(gamepad2.dpad_up) robot.claw_level.setPosition(robot.claw_level.getPosition() + 0.03);

        if(gamepad2.dpad_down) robot.claw_level.setPosition(robot.claw_level.getPosition() - 0.03);


        if(gamepad2.dpad_left)  robot.claw_rotate.setPosition(0); //think this is horizontal to robot

        if(gamepad2.dpad_right) robot.claw_rotate.setPosition(0.5); //and i think this is vertical to the robot

        if(gamepad2.left_trigger>1) robot.claw.setPosition(0.5); //130 degrees from 180 (closed)

        if(gamepad2.right_trigger>1) robot.claw.setPosition(0); // Full open

        // update prevX
        prevX = gamepad1.x;    }

    @Override
    public void stop() {
        robot.left_front_drive.setPower(0);
        robot.left_back_drive.setPower(0);
        robot.right_front_drive.setPower(0);
        robot.right_back_drive.setPower(0);

        robot.main_arm.setPower(0);
        robot.slide.setPower(0);
        /*
        claw_rotate.setPosition(0);
        claw.setPosition(0);
        */
    }
}
