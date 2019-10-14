package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;
import org.firstinspires.ftc.teamcode.Methods.*;
import org.firstinspires.ftc.teamcode.Variables.*;



@TeleOp(name="Tank TeleOp", group="TeleOp")

public class TeleOpTank extends OpMode {
    //Crates HardwareMap object robot
    HardwareMapMain robot = new HardwareMapMain();
    GeneralMethods methods = new GeneralMethods();
    Reference ref = new Reference();
    
    //method for servo change
    private void servoChange(String servo, double posChange) {
        switch(servo){
            case "claw":
                robot.claw.setPosition(robot.claw.getPositon() + posChange);
                break;
            case "rotate":
                robot.claw_rotate.setPosition(robot.claw_rotate.getPosition() + posChange);
                break;
        }
    }
    
    //method for motor change (not drive)
    private void motorChange(String motor, double posChange){
        switch(motor){
            case "arm":
                robot.main_arm.setPower(posChange);
                break;
            case "slide":
                robot.slide1.setPower(posChange);
                robot.slide2.setPower(
                break;
        }
    }
                    
                   
                    
                    
                  
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
         * Trigger LT: open claw
         * Trigger RT: close claw
         *
         */
        /*Things on claw current:
        robot.main_arm.setPower();
        robot.claw_level.setPosition();
        robot.claw.setPosition();
         */
        // Obtain joystick values
        //  robot.hangerServo.setPosition(0);
        double rightDrive = -gamepad1.right_stick_y;
        double leftDrive = -gamepad1.left_stick_y;
        boolean clawRotateLeft = gamepad2.dpad_left;
        boolean clawRotateRight = gamepad2.dpad_right;
        float armMove = gamepad.right_stick_y;
        float clawOpen = gamepad.left_trigger;
        float clawClosed = gamepad.right_trigger;
        boolean slideUp = gamepad.y;
        boolean slideDown = gamepad.a;
        
        

        // Set joystick values to motor values on robot
        robot.left_front_drive.setPower(leftDrive);
        robot.left_back_drive.setPower(leftDrive);
        robot.right_front_drive.setPower(rightDrive);
        robot.right_back_drive.setPower(rightDrive);
        
       
        
        if(armMove != 0.0){
            while(armMove != 0.0){
                motorChange("arm", 0.5 * ref.motorDegreesConst);
                //put something for claw leveler
            }
        }
        
        if(clawOpen != 0.0 || clawClosed != 0.0){
            while(clawOpen != 0.0){
                servoChange("claw", 0.5 * ref.servoDegreesConst);
                clawOpen = gamepad.left_trigger;
            }
            while(clawClosed != 0.0){
                servoChange("claw", 0.5 * ref.servoDegreesConst);
                clawClosed = gamepad.right_trigger;
            }
        }
        
        
        if( clawRotateLeft == true || clawRotateRight == true){
            while(clawRotateLeft) {
                servoChange("rotate", 0.5);
                clawRotateLeft = gamepad1.dpad_left;
            }
            while(clawRotateRight){
                servoChange("rotate", 0.5);
                clawRotateRight = gamepad1.dpad_right;
            }
        }
        
        if(slideUp != 0.0){
            while(slideUp != 0.0){
                motorChange("slide", 0.5 * ref.motorDegreesConst);
                slideUp = gamepad.y;
            }
        }
        
        
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
