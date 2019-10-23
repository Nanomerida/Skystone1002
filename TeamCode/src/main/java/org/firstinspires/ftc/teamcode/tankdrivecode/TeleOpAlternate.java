package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;
import org.firstinspires.ftc.teamcode.Methods.*;
import org.firstinspires.ftc.teamcode.Variables.*;


@TeleOp(name="TeleOpAlternate", group="TeleOp")
public class TeleOpAlternate extends OpMode {

    HardwareMapMain robot = new HardwareMapMain();
    GeneralMethods methods = new GeneralMethods();
    Reference ref = new Reference();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Obtain joystick values
        //  robot.hangerServo.setPosition(0);
        double right = -gamepad1.right_stick_y;
        double left = -gamepad1.left_stick_y;

        // Set joystick values to motor values on robot
        robot.left_front_drive.setPower(left);
        robot.left_back_drive.setPower(left);
        robot.right_front_drive.setPower(right);
        robot.right_back_drive.setPower(right);

        //DOES THIS WORK?
        robot.main_arm.setPower(gamepad2.right_stick_y);

        if(gamepad2.a){
            robot.slide.setPower(-1.0);
        }

        if(gamepad2.y){
            robot.slide.setPower(1.0);
        }

        if(gamepad2.dpad_left){
            robot.claw_rotate.setPosition(0);
        }

        if(gamepad2.dpad_right){
            robot.claw_rotate.setPosition(1);
        }

        if(gamepad2.left_bumper){
            robot.claw.setPosition(1);
        }

        if(gamepad2.right_bumper){
            robot.claw.setPosition(0);
        }
    }

    @Override
    public void stop() {
        robot.left_front_drive.setPower(0);
        robot.left_back_drive.setPower(0);
        robot.right_front_drive.setPower(0);
        robot.right_back_drive.setPower(0);

        robot.main_arm.setPower(0);
        robot.slide.setPower(0);
        robot.claw_rotate.setPosition(0);
        robot.claw.setPosition(0);
    }
}