package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CRVuforia.VuforiaBlue;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.firstinspires.ftc.teamcode.Mecanum.CRPosition.*;

import java.util.ArrayList;
@Autonomous(name = "RedOdometry", group = "Mecanum")
public class RedOdometry extends LinearOpMode {

    VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file
    private ElapsedTime refreshTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public DcMotor left_front_drive   = null;
    public DcMotor  left_back_drive  = null;
    public DcMotor  right_front_drive = null;
    public DcMotor  right_back_drive = null;

    public WebcamName webcam = null;
    ExpansionHubEx expansionHub10;
    RevBulkData bulkData;
    ExpansionHubMotor left_y_encoder, right_y_encoder, x_encoder = null;


    //IMU STUFF
    public BNO055IMU imu;
    private Orientation angles;

    private static int stonePos;
    private static final boolean redSide = true;
    private ArrayList<DcMotor> driveMotors = new ArrayList<DcMotor>();
    private ArrayList<ExpansionHubMotor> encoders = new ArrayList<>();

    //Our odometry instance
    protected CROdometry odometry;

    //Telemetry items
    Telemetry.Item currentGoal;





    public double degreesConversion(){
        while(refreshTimer.milliseconds() < 3){sleep(1);}
        refreshTimer.reset();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double theta = this.angles.firstAngle;
        if(redSide) {
            if (theta < 0) theta += 270;
            else theta -= 90;
        }
        theta = AngleUnit.normalizeDegrees(theta);
        if(theta < 0) theta += 360;
        return theta;
    }

    public boolean goToPos(double x, double y){

        currentGoal.setValue("%.3f, %.3f", x, y);
        try {
            while (odometry.MoveOdomPosition(x, y, degreesConversion()) && opModeIsActive()) {

            }
        } catch (Exception e){
            telemetry.addData("ODOMETRY ERROR!!!!!!", e.getLocalizedMessage());
            telemetry.update();
            return false;
        }
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);
        return true;

    }



    @Override
    public void runOpMode() throws InterruptedException {


        //Initialize motors
        left_front_drive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");


        //Expansion Hub with encoders
        expansionHub10 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 10");

        //External encoders
        left_y_encoder = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "left_y_encoder");
        right_y_encoder = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "right_y_encoder");
        x_encoder = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "x_encoder");


        //Webcam
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //Initialize vuforia with webcam
        blockPosBlue.blueInit(webcam);



        //Set up the drive base
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //reset the external encoders
        left_y_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_y_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        x_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_y_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_y_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        x_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //reverse the opposite drive motors
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);

        right_y_encoder.setDirection(ExpansionHubMotor.Direction.REVERSE);


        //adds motors to ArrayList
        driveMotors.add(left_front_drive);
        driveMotors.add(left_back_drive);
        driveMotors.add(right_front_drive);
        driveMotors.add(right_back_drive);


        //add encoders
        encoders.add(left_y_encoder);
        encoders.add(right_y_encoder);
        encoders.add(x_encoder);


        // MORE IMU STUFF

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        //Set up odometry
        odometry = new CROdometry(this, expansionHub10, encoders, new double[] {0,0}, degreesConversion(), driveMotors);

        currentGoal = telemetry.addData("Current Odometry Goal", "Waiting For Start!");





        waitForStart();


        sleep(2000);

        //odometry.MoveOdomPosition(36, 48, degreesConversion());

        while (!isStopRequested()){
            telemetry.addLine("DONE!!!!!!!!!!");
            telemetry.update();
        }



    }
}
