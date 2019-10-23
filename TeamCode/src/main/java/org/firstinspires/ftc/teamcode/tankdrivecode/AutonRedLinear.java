package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous (name = "AutonRedLinear", group = "Autonomous")

public class AutonRedLinear extends LinearOpMode {


    private HardwareMapMain robot   = new HardwareMapMain();
    private GeneralMethods methods = new GeneralMethods();
    private ElapsedTime timer = new ElapsedTime(0);


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

    /* Arm */
    public void armMove(int armPower, double clawPower){ //convert degrees to Servo powers
        clawPower = methods.degreeServoConv(clawPower);
        robot.main_arm.setPower(0.5);
        robot.main_arm.setTargetPosition(armPower);
        robot.claw_level.setPosition(clawPower);
    }

    private boolean driveBusy(){
        boolean busy = false;

        if(robot.left_front_drive.isBusy() || robot.left_back_drive.isBusy() || robot.right_front_drive.isBusy() || robot.right_back_drive.isBusy()){
            busy = true;
        }
        return busy;
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

    public void turnDrive(String direction, double power, double degrees) {


        String cc = "cw";

        double inches = (degrees * degreesToRadians) * ROBOT_WHEEL_DIST_INCHES;

        if(direction.equals(cc)){
            robot.right_front_drive.setDirection(DcMotor.Direction.REVERSE);
            robot.right_back_drive.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            robot.left_front_drive.setDirection(DcMotor.Direction.REVERSE);
            robot.left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        }

        robot.resetEncoders();


        //set desired power
        robot.left_front_drive.setPower(power);
        robot.left_back_drive.setPower(power);
        robot.right_front_drive.setPower(power);
        robot.right_back_drive.setPower(power);

        //TURN
        robot.left_front_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        robot.left_back_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        robot.right_front_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));
        robot.right_back_drive.setTargetPosition((int) round(inches / COUNTS_PER_INCH));

        robot.setRunToPosition();
    }



    /**Make sure these measurments are correct*/

    static final double     ROBOT_WHEEL_DIST_INCHES = 8.5f;     // distance from center of robot to wheels
    static final double     COUNTS_PER_WHEEL_REV    = 96 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_MM       = 96 ;     // For figuring circumference
    static final float     COUNTS_PER_INCH         = 2.9452f;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /* Other Variables */
    public static final double degreesToRadians = 180.0 / Math.PI;
    private VuforiaRed blockPosRed = new VuforiaRed();
    private Reference ref = new Reference();
    public static final double servoDegreesConst = 0.005;
    public static final double clawClosed = 45.0d;
    public static final double clawOpen = 90.0d;
    public static final String driveIdle = "IDLE"; /**/
    public static final String driveMoving = "MOVING"; /**/
    public static final String driveTurning = "TURNING"; /**/

    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;
    //NOTE: to get heading, do degreesConversion()

    @Override
    public void runOpMode(){

        int skystonePos = 4;

        blockPosRed.redInit(); //sets up vuforia


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


        robot.init(hardwareMap);
        robot.resetEncoders(); //obvious


        /*DO NOT DELETE!!!!!!!!!!!! If deleted, robot will automatically navigate to opponent's Capstone!!!!! */
        telemetry.addData("Say", "The Matrix is Ready");
        telemetry.addData("Glitches detected:", "0");
        telemetry.update();
        timeDelay(2.0f);


        telemetry.addData("Calculating Risk of Vuforia AI Taking Control .......", "....");
        telemetry.addData("Risk calculated:", ref.vuforiaRisk);
        telemetry.update();
        timeDelay(2.0f);


        waitForStart();


        //Steps
        moveDrive(0.8, 26.5f); // move forward to skystone

        robot.resetEncoders();
        int testResult = blockPosRed.visionTest();
        switch (testResult){ //vuforia
            case 0: //towards bridge
                //move there
                robot.main_arm.setTargetPosition(280); //90 degrees down
                robot.claw_level.setPosition(90.0 * servoDegreesConst); //claw level at 90 to match arm
                while(robot.main_arm.isBusy()) {
                    sleep(1);
                }//wait for arm
                robot.claw.setPosition(90.0 * servoDegreesConst); //open claw
                moveDrive(0.5, 6.5f);
                while(driveBusy()){
                    sleep(1);
                }

                skystonePos = 0;
                break;
            case 1: //center
                //move there
                robot.main_arm.setTargetPosition(280); //90 degrees down
                robot.claw_level.setPosition(90.0 * servoDegreesConst); //claw level at 90 to match arm
                while(robot.main_arm.isBusy()) {
                    sleep(1);
                }//wait for arm
                robot.claw.setPosition(90.0 * servoDegreesConst); //open claw
                turnDrive("ccw", 0.3, 20.0f);
                while (driveBusy()){
                    sleep(1);
                }
                moveDrive(0.5, 6.5f);
                while(driveBusy()){
                    sleep(1);
                }
                while(driveBusy()){
                    sleep(1);
                }
                skystonePos = 1;
                break;
            case 2: //towards wall
                robot.main_arm.setTargetPosition(280); //90 degrees down
                robot.claw_level.setPosition(90.0 * servoDegreesConst); //claw level at 90 to match arm
                while(robot.main_arm.isBusy()) {
                    sleep(1);
                }//wait for arm
                robot.claw.setPosition(90.0 * servoDegreesConst); //open claw
                turnDrive("cc", 0.3, 20);
                while(driveBusy()){
                    sleep(1);
                }
                moveDrive(0.5, 6.5f);
                while(driveBusy()){
                    sleep(1);
                }
                skystonePos = 2;
                break;
        }
        //each vuforia case should end at the same pos so they can be brought together for the next step.

        //next step
        turnDrive("cw", .5, 90);

        //and so on.
        moveDrive(1,69f);
        robot.claw.setPosition(90.0 * servoDegreesConst); //open claw

        moveDrive(-1, 69f);

        turnDrive("ccw",0.5, 90);
        switch (testResult){ //vuforia
            case 0:
                //Pick up stone 1
                break;
            case 1:
                //Pick up stone 2
                break;
            case 2:
                //pick up stone 1
                break;
        }

        turnDrive("cw", .5, 90);

        //and so on.
        moveDrive(1,69f);
        robot.claw.setPosition(90.0 * servoDegreesConst); //open claw

        moveDrive(-1, 40f); // park under Skybridge







    }



}
