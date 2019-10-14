package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; //IMU THINGS
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.NewVuforia;


import static java.lang.Math.cos; //Ryan's Math Stuff
import static java.lang.Math.sin;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

@Autonomous (name = "TankAutonomous", group = "Autonomous")

public class AutonomousTank extends OpMode {
    HardwareMapTank robot   = new HardwareMapTank();


    //define methods to be used here
    public double degreeServoConv(double degrees){
        return degrees * servoDegreesConst;
    }

    /*Moving claw to keep up with amr movement */

    public double armClawPower(double armPower){ //write orlando's math here for arm to claw_leveler power.
        telemetry.update();
        return armPower;
    }

    /* Moving Claw */
    public void armMove(double armPower, double clawPower){ //convert degrees to Servo powers
        clawPower = degreeServoConv(clawPower);
        robot.main_arm.setPower(armPower);
        robot.claw_level.setPosition(clawPower);
    }

    public void EncoderDrive(double speed, double leftInches, double rightInches, double timeoutS) throws InterruptedException { //nelitha's stuff
        int newLeftTarget;
        int newRightTarget;
        int newHangerTarget;
    }


    /**Make sure these measurments are correct*/
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /* Other Variables */
    private NewVuforia blockPos = new NewVuforia();
    public static final double servoDegreesConst = 0.005;
    public static final double clawClosed = 45.0d;
    public static final double clawOpen = 90.0d;
    public static final String driveIdle = "IDLE";
    public static final String driveMoving = "MOVING";
    public static final String driveTurning = "TURNING";

    //IMU STUFF
    BNO055IMU imu;
    Orientation angles;
    //NOTE: to get heading, do this.angles.firstAngle


    @Override
    public void init() {
        //Set up vuforia
        blockPos.runOpMode();
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
                this.angles.firstAngle);
        telemetry.update();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.resetEncoders(); //obvious
        //idle();  what is this for?
        robot.setRunWithoutEncoders(); //do i need to explain?

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();



        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition());
        telemetry.update();
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
         * -RyanD
         */

//---------------------------------------------------------------------------------------------------------------------


        //Telemetry stuff
        Telemetry.Item driveStatus = telemetry.addData("Drive Base Status:", driveIdle); //drive status
        Telemetry.Item armStatus = telemetry.addData("Arm Motor Status:", "IDLE");
        Telemetry.Item clawLevelStatus = telemetry.addData("Claw Level Servo Status:", "IDLE");
        Telemetry.Item clawStatus = telemetry.addData("Claw Servo Status:", "IDLE");
        Telemetry.Item visionStatus = telemetry.addData("Vision Testing Status:", "DISABLED"); //first item
        telemetry.update();
        //For Telemetry, use just do it whenever one of these actions is performed.

        /**THIS IS THE PART THAT NEEDS TO BE ADJUSTED PER EACH AUTON*/
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        /*
         *  Method to perfmorm a relative move, based on encoder counts.
         *  Encoders are not reset as the move is based on the current position.
         *  Move will stop if any of three conditions occur:
         *  1) Move gets to the desired position
         *  2) Move runs out of time
         *  3) Driver stops the opmode running.
         */


        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = robot.left_front_drive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newLeftBackTarget = robot.left_back_drive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newRightFrontTarget = robot.right_front_drive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newRightBackTarget = robot.right_back_drive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newHangerTarget = robot.hangerMotor.getCurrentPosition() + (int) (hangerInches * COUNTS_PER_INCH);
        robot.left_front_drive.setTargetPosition(newLeftFrontTarget);
        robot.left_back_drive.setTargetPosition(newLeftFrontTarget);
        robot.right_front_drive.setTargetPosition(newRightFrontTarget);
        robot.right_back_drive.setTargetPosition(newRightBackTarget);

        robot.setRunToPosition();


        // reset the timeout time and start motion.
        robot.leftFrontMotor.setPower(Math.abs(speed));
        robot.leftBackMotor.setPower(Math.abs(speed));
        robot.rightFrontMotor.setPower(Math.abs(speed));
        robot.rightBackMotor.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.leftFrontMotor.isBusy())) {

            // Allow time for other processes to run.
            idle();
        }

        // Stop all motion;
        robot.stopDrive();

        // Turn off RUN_TO_POSITION
        robot.setRunWithEncoders();

        sleep(250);   // optional pause after each move
    }
}