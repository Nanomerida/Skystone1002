package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name = "TeleOpMain", group="TeleOp")

public class TeleOpMain extends OpMode {

    TeleOpFieldCentric fieldCentric;
    ElapsedTime ping = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;
    public Servo claw = null;
    public DcMotorSimple lift_left = null;
    public DcMotorSimple lift_right = null;
    private ExpansionHubEx expansionHub1; //hub for motors
    private RevBulkData revBulkData1;


    private boolean prevX = false;
    private boolean prevY = false;

    private float clawOpenPos = 1; // Nelitha change these based on servo
    private float clawClosedPos = 0;

    float[] inputs;
    float[] outputs;
    float[] prevOutputs;


    enum DriveState {
        ULTRA_EPIC_FAST,
        FAST
    }

    enum LiftState {
        ULTRA_EPIC_FAST,
        FAST
    }

    private DriveState driveState = DriveState.ULTRA_EPIC_FAST;
    private LiftState liftState = LiftState.ULTRA_EPIC_FAST;

    
    

    private static float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }

    private boolean isDriveUpdateWorthy(){
        float total = 0;
        for(int i=0; i<3; i++){
            total += (outputs[i] - prevOutputs[i]);
        }
        total /= 4;
        return (Math.abs(total) >= 0.001);
    }

    private boolean isLiftUpdateWorthy(double previous, double update){
        return (Math.abs((update - previous)) >= 0.01);
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
        
        lift_left =  hardwareMap.get(DcMotorSimple.class, "lift_left");
        lift_right = hardwareMap.get(DcMotorSimple.class, "lift_right");

        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");


        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);


        lift_right.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }
    
    @Override
    public void loop() {


        revBulkData1 = expansionHub1.getBulkInputData();
        double currentPower = ((lift_left.getPower() + lift_right.getPower()) / 2);


        //Manipulator gamepad readings
        double liftPower = gamepad2.right_stick_y;
        boolean clawOpen = (gamepad2.left_bumper && claw.getPosition() != clawOpenPos);
        boolean clawClosed = (gamepad2.right_bumper && claw.getPosition() != clawClosedPos);

        //Turn slow mode off, if pressed and not already active
        driveState = (gamepad1.x && !prevX) ? DriveState.FAST : DriveState.ULTRA_EPIC_FAST;

        //The same for the lift
        liftState  = (gamepad2.y && !prevY) ? LiftState.FAST : LiftState.ULTRA_EPIC_FAST;



        inputs = new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x};



        //Calculate power for drive
        outputs = m_v_mult(matrix, inputs);





        if(isDriveUpdateWorthy()) {
            switch (driveState) {
                case ULTRA_EPIC_FAST:
                    left_front_drive.setPower(outputs[0]);
                    left_back_drive.setPower(outputs[1]);
                    right_front_drive.setPower(outputs[2]);
                    right_back_drive.setPower(outputs[3]);

                    break;

                case FAST:
                    left_front_drive.setPower(outputs[0] * 0.5f);
                    left_back_drive.setPower(outputs[1] * 0.5f);
                    right_front_drive.setPower(outputs[2] * 0.5f);
                    right_back_drive.setPower(outputs[3] * 0.5f);

                    break;
            }
        }
        if(isLiftUpdateWorthy(currentPower, liftPower)) {
            switch (liftState) {
                case ULTRA_EPIC_FAST:
                    lift_left.setPower(liftPower);
                    lift_right.setPower(liftPower);

                    break;

                case FAST:
                    lift_left.setPower(liftPower * 0.5f);
                    lift_right.setPower(liftPower * 0.5f);

                    break;
            }
        }



        
        //Control claw
        if(clawOpen) claw.setPosition(clawOpenPos);
        else if(clawClosed) claw.setPosition(clawClosedPos);

        //store current slow mode statuses
        prevX = gamepad1.x;
        prevY = gamepad2.y;

        //store this iteration's outputs
        prevOutputs = outputs.clone();

        //Useful telemetry
        telemetry.addData("Motor Velocities" , ":");
        telemetry.addData("Left Front:", "%2.2f", left_front_drive.getPower());
        telemetry.addData("Left Back:", "%2.2f", left_back_drive.getPower());
        telemetry.addData("Right Front:", "%2.2f", right_front_drive.getPower());
        telemetry.addData("Right Back:", "%2.2f",right_back_drive.getPower());

        
        //update telemetry
        telemetry.update();


        //loopNum++;


    }
    @Override
    public void stop() {
        //Stop drive motors
        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);
        lift_left.setPower(0);
        lift_right.setPower(0);
    }
}
