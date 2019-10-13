package org.firstinspires.ftc.teamcode.OKIIIcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;

@TeleOp(name="Outreach TeleOp", group="TeleOp")
//@Disabled
public class TeleOpOutreach extends OpMode {
    //Creates HardwareMap object robot
    private HardwareMapMain robot = new HardwareMapMain();
    @Override
    public void init() {
        //Initializes with the hardwareMap
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
    }


    //Ends all motor actions by setting power to 0 when teleOp is finished
    @Override
    public void stop() {
        robot.left_front_drive.setPower(0);
        robot.left_back_drive.setPower(0);
        robot.right_front_drive.setPower(0);
        robot.right_back_drive.setPower(0);

    }
}