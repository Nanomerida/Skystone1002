package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Methods.MecMoveProcedureStorage;
import java.util.HashMap;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name = "TeleOpMain", group="TeleOp")

public class TeleOpMain extends OpMode {

    TeleOpFieldCentric fieldCentric;

    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;
    public Servo claw = null;
    public ExpansionHubMotor lift = null;
    private ExpansionHubEx expansionHub1; //hub for motors
    private ExpansionHubEx expansionHub10; //hub for others
    private RevBulkData revBulkData1;
    private RevBulkData revBulkData10;
    
    private boolean slowModeOn = false;
    //private boolean fieldCentricOn = false;
    private boolean prevX = false;
    //private boolean prevY = false;

    private float clawOpenPos = 1; // Nelitha change these based on servo
    private float clawClosedPos = 0;

    //public BNO055IMU imu;
    float[] inputs;

    int loopNum = 0;
    
    

    private float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }
    
    //Array to hold movement instructions
    private float[][] matrix = {{0.55f, 1.0f, 0.5f, 1.0f}, {0.4f, -0.95f, -0.45f, 0.95f}, {0.65f, 1.0f, -0.65f, -1.0f}};

    //Initializes with the hardwareMap
    @Override
    public void init() {

        left_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "right_back_drive");


        claw = hardwareMap.get(Servo.class, "claw");
        
        lift = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "lift");

        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHub10 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 10");


        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);


        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /* BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters); */
        //fieldCentric = new TeleOpFieldCentric(imu);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //expansionHub10.setLedColor(255, 0, 0);
        //expansionHub1.setLedColor(0, 255, 0);
    }
    
    @Override
    public void loop() {

        /*switch (loopNum){
            case 1:         expansionHub10.setLedColor(255, 0, 0); expansionHub1.setLedColor(0, 255, 0); break;
            case 2:        expansionHub10.setLedColor(0, 255, 0); expansionHub1.setLedColor(0, 0, 255); break;
            case 3:         expansionHub10.setLedColor(0, 0, 255); expansionHub1.setLedColor(255, 0, 0); break;
            case 4: loopNum = 1; break;
        } */

        revBulkData1 = expansionHub1.getBulkInputData();
        revBulkData10 = expansionHub10.getBulkInputData();

        //Manipulator gamepad readings
        double liftPower = (gamepad2.right_stick_y != 0&& revBulkData10.getMotorCurrentPosition(lift) != 0)? gamepad2.right_stick_y : 0;
        boolean clawOpen = (gamepad2.left_bumper && claw.getPosition() != clawOpenPos);
        boolean clawClosed = (gamepad2.right_bumper && claw.getPosition() != clawClosedPos);

        //Turn slow mode of, if pressed and not already active
        if(gamepad1.x && !prevX) slowModeOn = !slowModeOn;
        float speed = (slowModeOn) ? 0.5f : 1.0f;
        //if(gamepad1.y && !prevY) fieldCentricOn =  !fieldCentricOn;



        //Read from controller
        float[] inputs = {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x};
        //if(fieldCentricOn) inputs = fieldCentric.driveFieldRelative(-inputs[1], inputs[0], inputs[2]);

        
        //Calculate power for drive
        float[] outputs = m_v_mult(matrix, inputs);
        
        //Set power to drive
        left_front_drive.setPower(outputs[0] * speed);
        left_back_drive.setPower(outputs[1] * speed);
        right_front_drive.setPower(outputs[2] * speed);
        right_back_drive.setPower(outputs[3] * speed);


        lift.setPower(liftPower);
        
        //Control claw
        if(clawOpen) claw.setPosition(clawOpenPos);
        else if(clawClosed) claw.setPosition(clawClosedPos);
        //else claw.setPower(0);

        //store current slow mode status
        prevX = gamepad1.x;

        //Useful telemetry
        telemetry.addData("Motor Velocities" , ":");
        telemetry.addData("Left Front:", revBulkData1.getMotorVelocity(left_front_drive));
        telemetry.addData("Left Back:", revBulkData1.getMotorVelocity(left_back_drive));
        telemetry.addData("Right Front:", revBulkData1.getMotorVelocity(right_front_drive));
        telemetry.addData("Right Back:", revBulkData1.getMotorVelocity(right_back_drive));


        telemetry.addData("Slow Mode:", slowModeOn);
        
        //update telemetry
        telemetry.update();

        loopNum++;


    }
    @Override
    public void stop() {
        //Stop drive motors
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);
        //arm.setPower(0);
        //claw.setPower(0);
    }
}
