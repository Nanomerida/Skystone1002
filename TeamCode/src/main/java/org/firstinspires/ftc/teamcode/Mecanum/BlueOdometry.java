package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CRVuforia.Vuforia;
import org.openftc.revextensions2.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.Mecanum.CRPosition.*;
import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.*;

@Autonomous(name = "BlueOdometry", group = "Mecanum")
public class BlueOdometry extends LinearOpMode {

    Vuforia blockPos = new Vuforia(); //creates an instance of the vuforia blue side file

    public WebcamName webcam = null;
    ExpansionHubEx expansionHub10;



    //IMU STUFF
    public BNO055IMU imu;
    private Orientation angles;

    private static final boolean redSide = false;

    //Our subsystems
    private CROdometry odometry;
    private MecanumIntake intake;
    private FoundationMover foundationMover;

    //Telemetry items
    Telemetry.Item currentPosGoal;
    Telemetry.Item currentPos;
    Telemetry.Item currentAngleGoal;
    Telemetry.Item currentAngle;

    /**Define starting info here! */
    public Pose2d globalPos = new Pose2d(0,0, 0);


    private static final double[] left_stone_pos = {-25,0};
    private static final double[] right_stone_pos = {-25,0};
    private static final double[] unseen_stone_pos = {-25,0};
    private static final double[] sensing_pos = {0,0};

    private Vuforia.SkystonePositon skystonePositon = Vuforia.SkystonePositon.UNKNOWN;



    public double degreesConversion(){
        double theta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        theta = AngleUnit.normalizeDegrees(theta - 90);
        return theta;
    }


    public boolean goToPos(double x, double y, long delay){

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
        sleep(delay);
        return true;

    }

    public boolean goToAngle(double angle, long delay){

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

        sleep(delay);
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
        intake = new MecanumIntake( hardwareMap);
        foundationMover = new FoundationMover(hardwareMap);

        odometry.init();
        intake.init();
        foundationMover.init();
        blockPos.blueInit(webcam, this);


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



        waitForStart();

        /* Main body of code here

            To do positional change:
                - goToPos(<x>, <y>);
                -^returns true if completed
            To do rotational change:
                - goToAngle(<angle>);
                -^returns true if completed


            IMPORTANT NOTE:
            All the angles are in euler form (-180, 180). The 0 is firmly on the positive
            y-axis (middle of field, parallel to alliance stations)
         */



        sleep(500);

        //goToPos(36, 48);
        //goToAngle(270);

        //Go to a generic position to turn robot
        goToPos(-48, -48, 500);

        //Rotate to face skystone
        goToAngle(-90, 500);

        goToPos(sensing_pos[0], sensing_pos[1], 200);

        findSkystone();

        switch (skystonePositon){
            case LEFT: getSkystoneLeft(); break;
            case RIGHT: getSkystoneRight(); break;
            case UNSEEN: getSkystoneUnseen(); break;
            case UNKNOWN: break;

            default: //CRY IN AGONY!!!!!!!
        }

        /*Go in front of bridge */
        goToPos(0,0,500);

        /* Turn towards bridge */
        goToAngle(0.1, 200);

        /*Go under bridge */
        goToPos(0,0,500);

        /*Release stone */
        intake.openClaw();
        sleep(1000);

        /*Go to park */
        goToPos(0,0,500);

        //That's it for now



        while (!isStopRequested()){
            telemetry.addLine("DONE!!!!!!!!!!");
            telemetry.update();
        }



    }


    private void findSkystone(){
        try {
            skystonePositon = blockPos.testVision();
        }catch (NullPointerException e){
            telemetry.addLine("VUFORIA ERROR!!!!!!");
            telemetry.update();
        }
    }


    private void getSkystoneLeft(){

        goToPos(left_stone_pos[0], left_stone_pos[1], 1000);

        intake.closeClaw();
        sleep(500);

        intake.armUp();
        sleep(500);

    }

    private void getSkystoneRight(){

        goToPos(right_stone_pos[0], right_stone_pos[1], 1000);

        intake.closeClaw();
        sleep(500);

        intake.armUp();
        sleep(500);

    }

    private void getSkystoneUnseen(){

        goToPos(unseen_stone_pos[0], unseen_stone_pos[1], 1000);

        intake.closeClaw();
        sleep(500);

        intake.armUp();
        sleep(500);

    }
}
