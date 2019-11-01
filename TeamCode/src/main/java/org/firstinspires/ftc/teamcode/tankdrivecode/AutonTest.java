/*NOTES:
Rev Core Hex: 2240 Counts per rotation

Formula for Counts per Inch:(COUNTS_PER_REVOLUTION_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_DIAMETER_INCHES * 3.1415)

Value for setTarget Position:
(Distance (inches)/ wheel circumfrence) * (motor ticks)
3/3.5 * 2240 <----- Lines 55-56



*/
package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Auton Test", group = "Autonomous")

public class AutonTest extends LinearOpMode {

    public DcMotor left_drive = null;
    public DcMotor right_drive = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double ROBOT_WHEEL_DIST_INCHES = 14.3f;     // distance from center of robot to wheels
    static final double COUNTS_PER_REVOLUTION_REV = 2240;// eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.5 ;
    static final double WHEEL_DIAMETER_INCHES = 3.50;     // For figuring circumference
    static final double COUNTS_PER_INCH = 101.859163578;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    /* Other Variables */
    public static final double degreesToRadians = 180.0 / Math.PI;



    @Override
    public void runOpMode() {

        left_drive  = hardwareMap.get(DcMotor.class, "leftDrive");
        right_drive = hardwareMap.get(DcMotor.class, "rightDrive");

//set target position, power, set run to position, wait for drive
        waitForStart();

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Should move 3 inches
        left_drive.setTargetPosition(1920);
        right_drive.setTargetPosition(1920);

        left_drive.setPower(1);
        right_drive.setPower(1);

        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left_drive.setPower(0);
        right_drive.setPower(0);
    }

}
