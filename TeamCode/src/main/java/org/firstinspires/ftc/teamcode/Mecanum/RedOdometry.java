package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CRVuforia.VuforiaBlue;
import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.MecanumIntake;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.firstinspires.ftc.teamcode.Mecanum.CRPosition.*;

import java.util.ArrayList;
@Autonomous(name = "RedOdometry", group = "Mecanum")
public class RedOdometry extends LinearOpMode {

    VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file
    private ElapsedTime refreshTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


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
    private CROdometry odometry;
    private MecanumIntake intake;
    private FoundationMover foundationMover;

    //Telemetry items
    Telemetry.Item currentPosGoal;
    Telemetry.Item currentPos;
    Telemetry.Item currentAngleGoal;
    Telemetry.Item currentAngle;

    /**Define starting info here! */
    public Pose2d globalPos = new Pose2d(0,0,180);





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

        currentPosGoal.setValue("%.3f, %.3f", x, y);
        try {
            while (odometry.MoveOdomPosition(x, y, degreesConversion()) && opModeIsActive()) {
                telemetry.update();
            }
        } catch (Exception e){
            telemetry.addData("ODOMETRY ERROR!!!!!!", e.getLocalizedMessage());
            telemetry.update();
            return false;
        }
        return true;

    }

    public boolean goToAngle(double angle){

        currentAngleGoal.setValue(angle);
        double previousAngle = degreesConversion();
        try {
            while (odometry.MoveAngle(angle , new double[] {previousAngle, degreesConversion()}) && opModeIsActive()) {
                telemetry.update();
            }
        } catch (Exception e){
            telemetry.addData("ODOMETRY ERROR!!!!!!", e.getLocalizedMessage());
            telemetry.update();
            return false;
        }
        return true;
    }



    @Override
    public void runOpMode() throws InterruptedException {


        //Expansion Hub with encoders
        expansionHub10 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 10");
        expansionHub10.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FASTPLUS_1M);


        //Webcam
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //Initialize vuforia with webcam
        blockPosBlue.blueInit(webcam);


        // MORE IMU STUFF

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.GYRONLY;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        //Set up odometry
        odometry = new CROdometry(this, expansionHub10, globalPos, hardwareMap);
        intake = new MecanumIntake(this, hardwareMap);
        foundationMover = new FoundationMover(hardwareMap);

        odometry.init();
        intake.init();
        foundationMover.init();

        currentPosGoal = telemetry.addData("Current Odometry Goal", "Waiting For Start!");
        currentAngleGoal = telemetry.addData("Current Angle Goal", "Waiting For Start!");
        currentPosGoal.setRetained(true);
        currentAngleGoal.setRetained(true);

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

        telemetry.update();



        waitForStart();


        sleep(2000);

        /* Main body of code here

            To do positional change:
                - goToPos(<x>, <y>);
                -^returns true if completed
            To do rotational change:
                - goToAngle(<angle>);
                -^returns true if completed
         */

        sleep(2000);


        //goToPos(36, 48);
        //goToAngle(270);

        while (!isStopRequested()){
            telemetry.addLine("DONE!!!!!!!!!!");
            telemetry.update();
        }



    }
}
