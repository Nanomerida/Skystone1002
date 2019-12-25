package org.firstinspires.ftc.teamcode.Mecanum.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Mecanum.CRPosition.CROdometry;
import org.firstinspires.ftc.teamcode.Mecanum.CRPosition.Pose2d;
import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.Driver;
import org.firstinspires.ftc.teamcode.hardware.DriveBaseVectors;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp(name = "Position Recording", group = "Testing")
public class PositionFinder extends LinearOpMode {

    Driver driver;

    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;



    ExpansionHubEx expansionHub10;



    //IMU STUFF
    public BNO055IMU imu;
    private Orientation angles;



    private CROdometry odometry;


    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;

    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<>();




    /**Define starting info here! */
    public Pose2d globalPos = new Pose2d(0,0, 0);

    Telemetry.Item currentAngle;
    Telemetry.Item currentPos;


    //Array to hold movement instructions
    //private float[][] matrix = {{0.75f, 1.0f, 0.7f, 1.0f}, {0.8f, -0.95f, -0.85f, 0.95f}, {0.75f, 1.0f, -0.75f, -1.0f}};

    private float[][] matrix = DriveBaseVectors.arcadeDriveVectors;


    public double degreesConversion(){
        double theta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        theta = AngleUnit.normalizeDegrees(theta -90);
        return theta;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //Expansion Hub with encoders
        expansionHub10 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 10");
        expansionHub10.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FASTPLUS_1M);


        left_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "left_front_drive");
        left_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "left_back_drive");
        right_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "right_front_drive");
        right_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "right_back_drive");

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);


        // MORE IMU STUFF

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Set up odometry
        odometry = new CROdometry( expansionHub10, globalPos, hardwareMap);


        odometry.init();

        currentPos = telemetry.addData("Current Position", new Func<String>(){
            @Override public String value() {
                return "X: " + globalPos.getPos().getX() + " Y: " + globalPos.getPos().getY();
            }
        });

        currentAngle = telemetry.addData("Current Heading", new Func<String>() {
            @Override public String value(){
                return "Θ: " + globalPos.getHeading();
            }
        });

        driver = new Driver(gamepad1, driveMotors, prevLeftBumper, prevRightBumper);

        telemetry.update();

        waitForStart();


        while (opModeIsActive()){

            odometry.update(degreesConversion());


            driver.drive(m_v_mult(matrix, new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x}));

            telemetry.update();



            //store current slow mode status
            prevLeftBumper = gamepad1.left_bumper;
            prevRightBumper = gamepad1.right_bumper;

        }

    }


    private static float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }


}
