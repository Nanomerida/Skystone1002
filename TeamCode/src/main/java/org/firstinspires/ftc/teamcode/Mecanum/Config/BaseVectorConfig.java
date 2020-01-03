package org.firstinspires.ftc.teamcode.Mecanum.Config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.Driver;
import org.firstinspires.ftc.teamcode.Methods.Refresher;
import org.firstinspires.ftc.teamcode.hardware.DriveBaseVectors;

import java.util.ArrayList;

@TeleOp

public class BaseVectorConfig extends OpMode{


    Driver driver;

    public DcMotorEx left_front_drive = null;
    public DcMotorEx left_back_drive = null;
    public DcMotorEx right_front_drive = null;
    public DcMotorEx right_back_drive = null;


    private ArrayList<DcMotorEx> driveMotors = new ArrayList<>();

    private Boolean prevLeftBumper = false;
    private Boolean prevRightBumper = false;


    private float[][] matrix = DriveBaseVectors.arcadeDriveVectors;

    private static float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }


    private Refresher slowModeUpdate = () -> {
        //store current slow mode status
        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;
    };

    @Override
    public void init(){

        left_front_drive =  hardwareMap.get(DcMotorEx.class, "left_front_drive");
        left_back_drive =  hardwareMap.get(DcMotorEx.class, "left_back_drive");
        right_front_drive =  hardwareMap.get(DcMotorEx.class, "right_front_drive");
        right_back_drive =  hardwareMap.get(DcMotorEx.class, "right_back_drive");



        right_front_drive.setDirection(DcMotorEx.Direction.REVERSE);
        right_back_drive.setDirection(DcMotorEx.Direction.REVERSE);



        driveMotors.add(left_front_drive);
        driveMotors.add(left_back_drive);
        driveMotors.add(right_front_drive);
        driveMotors.add(right_back_drive);


        driver = new Driver(gamepad1, driveMotors, prevLeftBumper, prevRightBumper);

    }

    @Override
    public void loop(){

        driver.drive(m_v_mult(matrix, new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x}));


    }
}
