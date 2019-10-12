package org.firstinspires.ftc.teamcode.OKIIIcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Outreach TeleOp", group="TeleOp")
//@Disabled
public class TeleOp extends OpMode {
    HardwareMapMain robot = new HardwareMapMain();
    //Creates HardwareMap object robot
    /**
     * CRGreenHardwareMap robot = new CRGreenHardwareMap(telemetry);
     */
    //Initializes with the hardwareMap
    @Override
    public void init() {
        robot.init(HardwareMapMain);
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
        robot.leftFrontMotor.setPower(left);
        robot.leftBackMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);
        robot.rightBackMotor.setPower(right);
    }


    //Ends all motor actions by setting power to 0 when teleOp is finished
    @Override
    public void stop() {
        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);

    }
}