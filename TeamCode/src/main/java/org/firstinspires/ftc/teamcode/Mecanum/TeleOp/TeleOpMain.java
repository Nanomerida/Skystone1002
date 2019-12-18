package org.firstinspires.ftc.teamcode.Mecanum.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.ArrayList;

@TeleOp(name = "TeleOpMain", group="TeleOp")

public class TeleOpMain extends OpMode {

    TeleOpFieldCentric fieldCentric;
    ElapsedTime ping = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;
    private Servo claw = null;
    private CRServo arm = null;
    private DcMotorSimple lift_left = null;
    private DcMotorSimple lift_right = null;
    private ExpansionHubEx expansionHub1; //hub for motors
    private ExpansionHubEx expansionHub10;
    private RevBulkData revBulkData1;
    private RevBulkData revBulkData10;

    private ArrayList<DigitalChannel> liftLimitSwitchesTop = new ArrayList<>();
    private ArrayList<DigitalChannel> liftLimitSwitchesBottom = new ArrayList<>();

    private DigitalChannel left_top_switch;
    private DigitalChannel left_bottom_switch;
    private DigitalChannel right_top_switch;
    private DigitalChannel right_bottom_switch;


    private boolean prevX = false;


    float[] inputs;
    float[] outputs;


    /**
     * Enum to represent the current speed mode of the drive base.
     */
    enum DriveState {
        SUPER_ULTRA_TURBO ,
        ULTRA_EPIC_FAST,
        FAST
    }


    /**
     * Enum to represent the current state of the lift, whether it has reached the bottom, top, or
     * moving/down, or has stopped and needs power to keep it in place.
     *
     */
    enum LiftState{
        BOTTOM,
        TOP,
        MOVING_UP,
        MOVING_DOWN,
        STOPPED,
    }


    private DriveState driveState = DriveState.ULTRA_EPIC_FAST;
    private LiftState liftState = LiftState.BOTTOM;



    private static float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }


    
    //Array to hold movement instructions
    private float[][] matrix = {{0.55f, 1.0f, 0.5f, 1.0f}, {0.8f, -0.95f, -0.85f, 0.95f}, {0.65f, 1.0f, -0.65f, -1.0f}};

    //Initializes with the hardwareMap
    @Override
    public void init() {

        left_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "right_back_drive");


        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(CRServo.class, "arm");
        
        lift_left = hardwareMap.get(DcMotorSimple.class, "lift_left");
        lift_right = hardwareMap.get(DcMotorSimple.class, "lift_right");

        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHub10 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 10");

        left_top_switch = hardwareMap.get(DigitalChannel.class, "left_top_switch");
        left_bottom_switch = hardwareMap.get(DigitalChannel.class, "left_bottom_switch");
        right_top_switch = hardwareMap.get(DigitalChannel.class, "right_top_switch");
        right_bottom_switch = hardwareMap.get(DigitalChannel.class, "right_bottom_switch");

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);

        //Reverse the left side
        lift_left.setDirection(DcMotor.Direction.REVERSE);

        left_top_switch.setMode(DigitalChannel.Mode.INPUT);
        left_bottom_switch.setMode(DigitalChannel.Mode.INPUT);
        right_top_switch.setMode(DigitalChannel.Mode.INPUT);
        right_bottom_switch.setMode(DigitalChannel.Mode.INPUT);


        liftLimitSwitchesTop.add(left_top_switch);
        liftLimitSwitchesBottom.add(left_bottom_switch);
        liftLimitSwitchesTop.add(right_top_switch);
        liftLimitSwitchesBottom.add(right_bottom_switch);


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }
    
    @Override
    public void loop() {
        //Get the data for this iteration
        revBulkData1 = expansionHub1.getBulkInputData();

        liftState = LiftState.STOPPED;

        //See if any of the limit switches have been tripped
        for(DigitalChannel limitSwitch : liftLimitSwitchesBottom){
            if(revBulkData1.getDigitalInputState(limitSwitch)) {
                liftState = LiftState.BOTTOM;
            }
        }
        for(DigitalChannel limitSwitch : liftLimitSwitchesTop){
            if(revBulkData1.getDigitalInputState(limitSwitch)){
                liftState = LiftState.TOP;
            }
        }

        //Check to see if the lift has stopped in mid-air.
        if(gamepad2.dpad_up || !gamepad2.dpad_down || liftState != LiftState.TOP || liftState != LiftState.BOTTOM){
            liftState = LiftState.MOVING_UP;
        }
        else if(!gamepad2.dpad_up || gamepad2.dpad_down || liftState != LiftState.TOP || liftState != LiftState.BOTTOM){
            liftState = LiftState.MOVING_DOWN;
        }



        //Turn slow mode off, if pressed and not already active
        driveState = (gamepad1.x && !prevX) ? DriveState.FAST : DriveState.ULTRA_EPIC_FAST;



        //Read driver inputs
        inputs = new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x};



        //Calculate power for drive
        outputs = m_v_mult(matrix, inputs);




        //Move drive base
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

            //Move lift based on state
            switch (liftState){
                case TOP:
                    if(gamepad2.dpad_up || gamepad2.dpad_down){
                        liftState = LiftState.MOVING_DOWN;
                    }
                    break;
                case BOTTOM:
                    if(gamepad2.dpad_up || gamepad2.dpad_down){
                        liftState = LiftState.MOVING_UP;
                    }
                    break;
                case MOVING_UP:
                    lift_left.setPower(-0.5); lift_right.setPower(-0.5);

                    break;
                case MOVING_DOWN:
                    lift_left.setPower(0.1); lift_right.setPower(0.1);
                    break;
                case STOPPED:
                    lift_left.setPower(0.1); lift_right.setPower(0.05);
                    break;
            }

        //Move lift
        /*if(gamepad2.dpad_up) {lift_left.setPower(-0.5); lift_right.setPower(-0.5); }
        else if(gamepad2.dpad_down) {lift_left.setPower(0.1); lift_right.setPower(0.1); }
        else lift_left.setPower(0); lift_right.setPower(0); */


        
        //Control claw
        if(gamepad2.right_bumper) claw.setPosition(0);
        else if(gamepad2.left_bumper) claw.setPosition(0.4);

        //Move arm
        arm.setPower(-gamepad2.left_stick_y);

        //store current slow mode status
        prevX = gamepad1.x;



        //Useful telemetry
        telemetry.addData("Motor Velocities" , ":");
        telemetry.addData("Left Front:", revBulkData1.getMotorVelocity(left_front_drive));
        telemetry.addData("Left Back:",  revBulkData1.getMotorVelocity(left_back_drive));
        telemetry.addData("Right Front:",  revBulkData1.getMotorVelocity(right_front_drive));
        telemetry.addData("Right Back:", revBulkData1.getMotorVelocity(right_back_drive));

        
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
