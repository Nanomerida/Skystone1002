package org.firstinspires.ftc.teamcode.Mecanum.Config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.Driver;
import org.firstinspires.ftc.teamcode.Methods.Refresher;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

@TeleOp
@Config
public class BaseVectorTuning extends OpMode {


    @Config
    static class BaseVectors {


        public static float strafeR_LeftFront = 0.85f;
        public static float strafeR_LeftBack = -0.75f;
        public static float strafeR_RightFront = -0.85f;
        public static float strafeR_RightBack = 0.75f;



    }
    
    private static final String PID_VAR_NAME = "STRAFE VECTOR";

    private String catName;
    private CustomVariable catVar;

    private SampleMecanumDriveBase drive;

    private void addPidVariable() {
        catName = BaseVectors.getClass().getSimpleName();
        catVar = (CustomVariable) dashboard.getConfigRoot().getVariable(catName);
        if (catVar == null) {
            // this should never happen...
            catVar = new CustomVariable();
            dashboard.getConfigRoot().putVariable(catName, catVar);

            RobotLog.w("Unable to find top-level category %s", catName);
        }

        CustomVariable vectorVar = new CustomVariable();
        pidVar.putVariable("Forward Left", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return (double) BaseVectors.strafeR_LeftFront;
            }

            @Override
            public void set(Double value) {
                BaseVectors.strafeR_LeftFront = (float) value;
            }
        }));
        pidVar.putVariable("Back Left", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return (double) BaseVector.strafeR_LeftBack;
            }

            @Override
            public void set(Double value) {
                BaseVectors.strafR_RightFront = (float) value;
            }
        }));
        pidVar.putVariable("Forward Right", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return (double) BaseVectors.strafeR_RightFront;
            }

            @Override
            public void set(Double value) {
                BaseVectors.strafeR_RightFront
            }
        }));
        pidVar.putVariable("Back Right", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return (double) BaseVectors.strafeR_RightFront;
            }

            @Override
            public void set(Double value) {
                BaseVectors.strafeR_RightBack = (float) value;
            }
        }));

        catVar.putVariable(PID_VAR_NAME, pidVar);
        dashboard.updateConfig();
    }

    private void removePidVariable() {
        if (catVar.size() > 1) {
            catVar.removeVariable(PID_VAR_NAME);
        } else {
            dashboard.getConfigRoot().removeVariable(catName);
        }
        dashboard.updateConfig();
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


        matrix[1][0] = BaseVectors.strafeR_LeftFront;
        matrix[1][1] = BaseVectors.strafeR_LeftBack;
        matrix[1][2] = BaseVectors.strafeR_RightFront;
        matrix[1][3] = BaseVectors.strafeR_RightBack;

        driver.drive(m_v_mult(matrix, new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x}));


    }
}
