package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ACTUAL TELEOP-- wheels", group="teleop")
@Disabled


public class Teleopskuf extends LinearOpMode {

    public DcMotor left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;

    private boolean slowModeOn = false;

    private static float speedConst = 1.0f;

    @Override
    public void runOpMode(){
        left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");



        waitForStart();



        while(opModeIsActive()){

            boolean slowMode = gamepad1.b;

            if(slowMode && !slowModeOn){
                speedConst = 0.5f;
                slowModeOn = true;
            }
            if(slowMode && slowModeOn){
                speedConst = 1.0f;
                slowModeOn = false;
            }

            double rightDrive = -gamepad1.right_stick_y;
            double leftDrive = -gamepad1.left_stick_y;


            left_front_drive.setPower(leftDrive * speedConst);
            left_back_drive.setPower(leftDrive * speedConst);
            right_front_drive.setPower(rightDrive * speedConst);
            right_back_drive.setPower(rightDrive * speedConst);
        }

    }
}
