package org.firstinspires.ftc.teamcode.Mecanum.Config;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.databind.ser.Serializers;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.Driver;
import org.firstinspires.ftc.teamcode.Methods.Refresher;
import org.firstinspires.ftc.teamcode.hardware.DriveBaseVectors;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

@TeleOp
@Config
public class BaseVectorTuning extends OpMode {


    @Config
    static class BaseVectors {


        public static float forward_LeftFront, forward_LeftBack, forward_RightFront, forward_RightBack;
        public static float strafeR_LeftFront, strafeR_LeftBack, strafeR_RightFront, strafeR_RightBack;
        public static float turnCW_LeftFront, turnCW_LeftBack, turnCW_RightFront, turnCW_RightBack;



    }

    Driver driver;

    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;

    private Boolean prevLeftBumper = false;
    private Boolean prevRightBumper = false;


    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<>();


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

    public static float[][] matrix = {{0.75f, 1.0f, 0.7f, 1.0f}, {0.85f, -0.75f, -0.85f, 0.75f}, {0.75f, 1.0f, -0.75f, -1.0f}};


    /**Update the slow mode status
     *
     */
    private Refresher slowModeUpdate = () -> {
        //store current slow mode status
        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;
    };

    @Override
    public void init(){

        left_front_drive =  hardwareMap.get(ExpansionHubMotor.class, "left_front_drive");
        left_back_drive = hardwareMap.get(ExpansionHubMotor.class, "left_back_drive");
        right_front_drive = hardwareMap.get(ExpansionHubMotor.class, "right_front_drive");
        right_back_drive =  hardwareMap.get(ExpansionHubMotor.class, "right_back_drive");

        left_front_drive.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);


        right_front_drive.setDirection(ExpansionHubMotor.Direction.REVERSE);
        right_back_drive.setDirection(ExpansionHubMotor.Direction.REVERSE);


        driveMotors.add(left_front_drive);
        driveMotors.add(left_back_drive);
        driveMotors.add(right_front_drive);
        driveMotors.add(right_back_drive);


        driver = new Driver(gamepad1, driveMotors, prevLeftBumper, prevRightBumper);

        //Does the same as the above
        slowModeUpdate.refresh();

    }

    @Override
    public void loop(){

        matrix[0][0] = BaseVectors.forward_LeftFront;
        matrix[0][1] = BaseVectors.forward_LeftBack;
        matrix[0][2] = BaseVectors.forward_RightFront;
        matrix[0][3] = BaseVectors.forward_RightBack;
        matrix[1][0] = BaseVectors.strafeR_LeftFront;
        matrix[1][1] = BaseVectors.strafeR_LeftBack;
        matrix[1][2] = BaseVectors.strafeR_RightFront;
        matrix[1][3] = BaseVectors.strafeR_RightBack;
        matrix[2][0] = BaseVectors.turnCW_LeftFront;
        matrix[2][1] = BaseVectors.turnCW_LeftBack;
        matrix[2][2] = BaseVectors.turnCW_RightFront;
        matrix[2][3] = BaseVectors.turnCW_RightBack;

        driver.drive(m_v_mult(matrix, new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x}));


    }
}
