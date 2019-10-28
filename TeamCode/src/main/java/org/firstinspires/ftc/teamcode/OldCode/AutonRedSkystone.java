package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Methods.GeneralMethods;
import org.firstinspires.ftc.teamcode.Variables.Reference;
import org.firstinspires.ftc.teamcode.CRVuforia.VuforiaRed;
import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;


import static java.lang.Math.abs;
import static java.lang.Math.round;

@Autonomous (name = "TankAutonomous", group = "Autonomous")

public class AutonRedSkystone extends OpMode {
    private HardwareMapMain robot   = new HardwareMapMain();
    private GeneralMethods methods = new GeneralMethods();
    private ElapsedTime timer = new ElapsedTime(0);


    //define methods to be used here
    private void timeDelay(float delay){ //method for time delay
        timer.reset();
        while(timer.seconds() != delay ){
            boolean y = false; //i.e, do nothing
        }
    }

    public double degreesConversion(){
        double theta = this.angles.firstAngle;
        if(theta < 0) {
            theta += 360;
        }
        return theta;
    }

    /*Moving claw to keep up with arm movement */

    public double armClawPower(double armPower){ //write orlando's math here for arm to claw_leveler power.
        telemetry.update();
        return armPower;
    }

    /* Moving Claw */
    public void armMove(double armPower, double clawPower){ //convert degrees to Servo powers
        clawPower = methods.degreeServoConv(clawPower);
        robot.main_arm.setPower(armPower);
        robot.claw_level.setPosition(clawPower);
    }

    private void moveDrive(double power, float inches){
        robot.resetEncoders();
        robot.setRunToPosition();

        robot.left_front_drive.setPower(power);
        robot.left_back_drive.setPower(power);
        robot.right_front_drive.setPower(power);
        robot.right_back_drive.setPower(power);

        robot.left_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        robot.left_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        robot.right_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        robot.right_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
    }

    public void turnDrive(String direction, double power, float inches) {

        String cc = "cc";

        if(direction.equals(cc)){
            robot.right_front_drive.setDirection(DcMotor.Direction.REVERSE);
            robot.right_back_drive.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            robot.left_front_drive.setDirection(DcMotor.Direction.REVERSE);
            robot.left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        }
        //set desired power
        robot.left_front_drive.setPower(power);
        robot.left_back_drive.setPower(power);
        robot.right_front_drive.setPower(power);
        robot.right_back_drive.setPower(power);

        //reset encoders before turning
        robot.resetEncoders();
        robot.setRunToPosition();
        //TURN
        robot.left_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        robot.left_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        robot.right_front_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));
        robot.right_back_drive.setTargetPosition(round(inches / COUNTS_PER_INCH));

    }



    /**Make sure these measurments are correct*/
    static final double     COUNTS_PER_WHEEL_REV    = 96 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_MM       = 75 ;     // For figuring circumference
    static final float     COUNTS_PER_INCH         = 0.0966f;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /* Other Variables */
    private VuforiaRed blockPosRed = new VuforiaRed();
    private Reference ref = new Reference();
    public static final double servoDegreesConst = 0.005;
    public static final double clawClosed = 45.0d;
    public static final double clawOpen = 90.0d;
    public static final String driveIdle = "IDLE"; /**/
    public static final String driveMoving = "MOVING"; /**/
    public static final String driveTurning = "TURNING"; /**/
    public static int step = 0;

    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;
    //NOTE: to get heading, do degreesConversion()


    @Override
    public void init() {
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
        telemetry.addData("Status","IMU Initialized, Current Heading %4d",
                degreesConversion());
        telemetry.update();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.resetEncoders(); //obvious

        // Send telemetry message to signify robot waiting;



        // Send telemetry message to indicate successful Encoder reset
        // Send telemetry message to signify robot waiting;
            /*DO NOT DELETE!!!!!!!!!!!! If deleted, robot will automatically navigate to opponent's Capstone!!!!! */
            telemetry.addData("Say", "The Matrix is Ready");
            telemetry.addData("Glitches detected:", "0");
            telemetry.update();
            timeDelay(2.0f);


            telemetry.addData("Calculating Risk of Vuforia AI Taking Control .......", "....");
            telemetry.addData("Risk calculated:", ref.vuforiaRisk);
            telemetry.update();
            timeDelay(2.0f);


    }
    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
    }

    public void loop() {
//---------------------------------------------------------------------------------------------------------------------
        /* NOTE TO 1002 -----> When the vision test is needed, do int skystonePos = blockPos.visionTest();
         * This will output either 0(closest to bridge), 1(center), or 2(closest to wall).
         * Also need to set which side of the filed for Vuforia before each match.
         * -RyanD
         */

//---------------------------------------------------------------------------------------------------------------------


        if ( //check for motor movement
                !robot.left_front_drive.isBusy() || !robot.left_back_drive.isBusy() || !robot.right_front_drive.isBusy() || !robot.right_back_drive.isBusy() || robot.slide.isBusy() || robot.main_arm.isBusy()) {

            switch (step) {
                case 0: //first step (move)
                    moveDrive(1, 3.9f); // put in drive power and inches desired
                    break;
                case 1: //second step (vuforia)
                    robot.resetEncoders();
                    switch(blockPosRed.visionTest()){

                        case 0: //skystone near wall
                            //move
                    }
                    moveDrive(1, 32.6f);
                    break;
                case 2: //third step and so on....
                    robot.resetEncoders();
                    moveDrive(1, 29.6f);
                    break;
            }//NOTE: end each step with break;, and still need to make turn method and other controls.

            step++;
        }
    }
    @Override
    public void stop() {
        robot.stopDrive();
    }
}
