package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

@TeleOp(name = "TeleOpMain", group="TeleOp")

public class TeleOpMain extends OpMode {

    TeleOpFieldCentric fieldCentric;

    MecMoveProcedureStorage procedures = new MecMoveProcedureStorage();
    private HashMap<String, float[]> mecanum = procedures.getMecanum();

    public DcMotor left_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor right_back_drive = null;
    //public DcMotor arm = null;
    //public CRServo claw = null;
    public DcMotor lift = null;
    
    private boolean slowModeOn = false;
    private boolean fieldCentricOn = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private String driveStatus;

    public BNO055IMU imu;
    float[] inputs;
    
    

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

        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");


        //arm = hardwareMap.get(DcMotor.class, "arm");
        //claw = hardwareMap.get(CRServo.class, "claw");
        
        lift = hardwareMap.get(DcMotor.class, "lift");


        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);


        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        fieldCentric = new TeleOpFieldCentric(imu);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }
    
    @Override
    public void loop() {

        //Manipulator gamepad readings
        double armPower = gamepad2.right_stick_y;
        double clawPowerClose = -gamepad2.right_trigger;
        double clawPowerOpen = gamepad2.left_trigger;

        //Turn slow mode of, if pressed and not already active
        if(gamepad1.x && !prevX) slowModeOn = !slowModeOn;
        float speed = (slowModeOn) ? 0.5f : 1.0f;
        //if(gamepad1.y && !prevY) fieldCentricOn =  !fieldCentricOn;
        
        //Useful telemetry
        telemetry.addLine("Motor Powers | ")
                .addData("Drive Status:", driveStatus)
                .addData("Left Front:", left_front_drive.getPower())
                .addData("Left Back:", left_back_drive.getPower())
                .addData("Right Front:", right_front_drive.getPower())
                .addData("Right Back:", right_back_drive.getPower());


        telemetry.addLine("Arm Power | ")
                .addData("Arm Motor:", armPower);
        
        telemetry.addLine("Claw Power | ")
                .addData("Claw Servo Closing:", clawPowerClose)
                .addData("Claw Servo Opening:", clawPowerOpen);
        
        telemetry.addData("Slow Mode:", slowModeOn);


        //Read from controller
        float[] inputs = {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x};
        //if(fieldCentricOn) inputs = fieldCentric.driveFieldRelative(-inputs[1], inputs[0], inputs[2]);
        
        //Update telemetry if moving
        if(gamepad1.left_stick_y == 0.0 && gamepad1.right_stick_x == 0.0) driveStatus = "IDLE";
        else driveStatus = "MOVING";

        
        //Calculate power for drive
        float[] outputs = m_v_mult(matrix, inputs);
        
        //Set power to drive
        left_front_drive.setPower(outputs[0] * speed);
        left_back_drive.setPower(outputs[1] * speed);
        right_front_drive.setPower(outputs[2] * speed);
        right_back_drive.setPower(outputs[3] * speed);


        if (gamepad1.dpad_up) {
            telemetry.addData("Direction:", "up");
            lift.setPower(.5);
        } else if (gamepad1.dpad_down) {
            telemetry.addData("Direction:", "down");
            lift.setPower(-.5);
        } else {
            telemetry.addData("Direction:", "none");
            lift.setPower(0);
        }

        //Control arm
        //arm.setPower(0.5555 * armPower);
        
        //Control claw
        //if(clawPowerOpen != 0.0) claw.setPower(0.7*clawPowerOpen);
        //else if(clawPowerClose != 0.0) claw.setPower(0.8*clawPowerClose);
        //else claw.setPower(0);

        //store current slow mode status
        prevX = gamepad1.x;
        prevY = gamepad1.y;
        
        //update telemetry
        telemetry.update();


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
