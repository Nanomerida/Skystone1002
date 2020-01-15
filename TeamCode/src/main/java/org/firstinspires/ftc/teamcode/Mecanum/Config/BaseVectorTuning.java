package org.firstinspires.ftc.teamcode.Mecanum.Config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.CRRoadrunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.Driver;
import org.firstinspires.ftc.teamcode.Methods.Refresher;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

@TeleOp
@Config
public class BaseVectorTuning extends OpMode {


    public static double strafeR_LeftFront = 0.72f;
    public static double strafeR_LeftBack = -0.60;
    public static double strafeR_RightFront = -0.60f;
    public static double strafeR_RightBack = 0.72f;

    FtcDashboard dashboard = FtcDashboard.getInstance();







    Driver driver;

    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;





    private static float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }



    //Array to hold movement instructions
    //private float[][] matrix = {{0.75f, 1.0f, 0.7f, 1.0f}, {0.8f, -0.95f, -0.85f, 0.95f}, {0.75f, 1.0f, -0.75f, -1.0f}};

    public static float[][] matrix = {{1.0f, 1.0f, 1.0f, 1.0f}, {0.72f, -0.60f, -0.60f, 0.72f}, {0.75f, 1.0f, -0.75f, -1.0f}};


    /**Update the slow mode status
     *
     */

    @Override
    public void init(){

        left_front_drive =  hardwareMap.get(ExpansionHubMotor.class, "left_front_drive");
        left_back_drive = hardwareMap.get(ExpansionHubMotor.class, "left_back_drive");
        right_front_drive = hardwareMap.get(ExpansionHubMotor.class, "right_front_drive");
        right_back_drive =  hardwareMap.get(ExpansionHubMotor.class, "right_back_drive");

        left_front_drive.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front_drive.setDirection(ExpansionHubMotor.Direction.REVERSE);
        right_back_drive.setDirection(ExpansionHubMotor.Direction.REVERSE);



        driver = new Driver(gamepad1, hardwareMap);


    }
    

    @Override
    public void loop(){


        matrix[1][0] = (float) strafeR_LeftFront;
        matrix[1][1] = (float) strafeR_LeftBack;
        matrix[1][2] = (float) strafeR_RightFront;
        matrix[1][3] = (float) strafeR_RightBack;

        driver.update();
        driver.drive();


    }

    @Override public void stop(){


    }
}
