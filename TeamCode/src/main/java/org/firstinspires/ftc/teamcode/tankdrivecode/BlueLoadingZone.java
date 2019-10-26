package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Methods.GeneralMethods;
import org.firstinspires.ftc.teamcode.Variables.Reference;
import org.firstinspires.ftc.teamcode.VuforiaBlue;
import org.firstinspires.ftc.teamcode.VuforiaRed;
import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;


import static java.lang.Math.abs;
import static java.lang.Math.PI;
import static java.lang.Math.round;

@Autonomous (name = "BlueLoadingZone", group = "Autonomous")

public class BlueLoadingZone extends LinearOpMode {

    public DcMotor left_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor right_back_drive = null;
    public Servo    claw   = null;
    public DcMotor slide = null;
    public DcMotor main_arm = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double ROBOT_WHEEL_DIST_INCHES = 8.5f;     // distance from center of robot to wheels
    static final double COUNTS_PER_WHEEL_REV = 96;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_MM = 90;     // For figuring circumference
    static final float COUNTS_PER_INCH = 2.9452f;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    /* Other Variables */
    public static final double degreesToRadians = 180.0 / Math.PI;
    private VuforiaBlue blockPosBlue = new VuforiaBlue();
    private Reference ref = new Reference();
    public static final double SERVODEGREES = 0.005;

    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;
    //NOTE: to get heading, do degreesConversion()

    @Override
    public void runOpMode() {

        left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        main_arm = hardwareMap.get(DcMotor.class, "main_arm");
        slide = hardwareMap.get(DcMotor.class, "slide");

        claw = hardwareMap.get(Servo.class,"claw");





    }


}

















}
