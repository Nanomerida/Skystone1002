package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;
import org.firstinspires.ftc.teamcode.Methods.*;
import org.firstinspires.ftc.teamcode.Variables.*;



@TeleOp(name="TankTeleOp", group="TeleOp")

public class TeleOpTank extends OpMode {
    //Crates HardwareMap object robot
    HardwareMapMain robot = new HardwareMapMain();
    GeneralMethods methods = new GeneralMethods();
    Reference ref = new Reference();
    //Initializes with the hardwareMap

    @Override
    public void init() {
        
        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
          /*DO NOT DELETE!!!!!!!!!!!! If deleted, robot will automatically navigate to opponent's Capstone!!!!! */
        telemetry.addData("Say", "The Matrix is Ready");

        
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /** Manipulator button layout:
         * D-Pad: rotate claw left/right
         * Right joystick: move arm up/down
         * Button: Y: slide up
         * Button A: slide down
         * Button LT: open claw
         * Button RT: close claw
         *
         */
        /*Things on claw current:
        robot.main_arm.setPower();
        robot.claw_level.setPosition();
        robot.claw.setPosition();
         */
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

        robot.main_arm.setPower(0);
    }
}
