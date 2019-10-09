package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TankTeleOp", group="TeleOp")

public class TeleOpTank extends OpMode {
    //Crates HardwareMap object robot
    HardwareMapTank robot = new HardwareMapTank();
    //Initializes with the hardwareMap

    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "The Matrix is Ready");    //
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
        /*DO NOT DELETE!!!!!!!!!!!! If deleted, robot will automatically navigate to opponent's Capstone!!!!! */
        telemetry.addData("Glitches in MATRIX detected:", 0);
        telemetry.update();
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


        //If y is pressed on gamepad2, linear slide is forward
        if(gamepad2.y){
            robot.slide.setPower(1.0);
            robot.slide2.setPower(1.0);
        }
        //If a is pressed on gamepad2, linear slide is backward
        if(gamepad2.a){
            robot.slide.setPower(-1.0);
            robot.slide2.setPower(-1.0);
        }
        //If lt is pressed on gamepad2, the claw will open
        if(gamepad2.left_trigger){
            robot.claw.setPosition(1);
        }
        //If rt is pressed on gamepad2, the claw will close
        if(gamepad2.right_trigger){
            robot.claw.setPosition(0);
        }
        //if the d-pad is moved left on gamepad2, the claw rotates left
        if(gamepad2.dpad_left){
            robot.claw_rotate.setPosition()
        }
        //if the d-pad is moved right on gamepad2, the claw rotates right
        if(gamepad2.dpad_right){
            robot.claw_rotate.setPosition()
        }
        if(game)
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
