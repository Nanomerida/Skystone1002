package org.firstinspires.ftc.teamcode.Mecanum.TeleOp;

import android.content.res.Resources;
import android.nfc.NfcAdapter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Methods.Refresher;
import org.firstinspires.ftc.teamcode.Methods.Toggle;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.hardware.DriveBaseVectors;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.*;

import java.util.ArrayList;
import java.util.Random;

@TeleOp(name = "TeleOpMain", group="TeleOp")

public class TeleOpMain extends OpMode {

    TeleOpFieldCentric fieldCentric;
    Driver driver;
    FoundationMover foundationMover;
    ElapsedTime ping = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    /**
     * Deserialize the controls
     */
    //DriverConfig.DriverControls driverControls = DriverConfig.deserializeDriver();
    //DriverConfig.ManipulatorControls manipulatorControls = DriverConfig.deserializeManip();


    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;
    private Servo claw = null;
    private Servo arm = null;
    private DcMotorSimple lift_left = null;
    private DcMotorSimple lift_right = null;
    private ExpansionHubEx expansionHub1; //hub for motors
    private ExpansionHubEx expansionHub10;
    private RevBulkData revBulkData1;
    private RevBulkData revBulkData10;

    private DigitalChannel left_bottom_switch = null;
    private DigitalChannel right_bottom_switch = null;


    enum FoundationState {
        DOWN_FOR_SIZING,
        LOCKED_ON_FOUNDATION,
        FULLY_UP
    }
    enum ClawState {
        OPEN,
        CLOSED
    }

    enum LiftState {
        MOVING,
        HOLDING_AT_POSITION,
        AT_BOTTOM,
        MOVING_TO_BOTTOM

    }



    private FoundationState foundationMoverState = FoundationState.DOWN_FOR_SIZING;
    private ClawState clawState = ClawState.OPEN;
    private LiftState liftState = LiftState.AT_BOTTOM;






    /**
     * Toggle control the claw servo.
     */
    private Toggle clawToggle = () -> {

        if(gamepad2.left_bumper){
            switch (clawState){
                case OPEN:
                    claw.setPosition(0);
                    clawState = ClawState.CLOSED;
                    break;
                case CLOSED:
                    claw.setPosition(0.4);
                    clawState = ClawState.OPEN;
            }
        }

    };

    private Toggle foundationToggle = () -> {

        if(gamepad2.a){
            switch (foundationMoverState){
                case FULLY_UP:
                    foundationMover.down();
                    foundationMoverState = FoundationState.LOCKED_ON_FOUNDATION;

                    break;
                case LOCKED_ON_FOUNDATION:
                    foundationMover.up();
                    foundationMoverState = FoundationState.FULLY_UP;

                    break;
                    //This only happens once in teleOp. Starting position
                case DOWN_FOR_SIZING:
                    foundationMoverState = FoundationState.FULLY_UP;
            }
        }
    };

    private Toggle armToggle = () -> {
        if(gamepad2.a){
            arm.setPosition(0);
        }
        else if(gamepad2.b){
            arm.setPosition(0.25);
        }
    };




    private static float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }




    //Array to hold movement instructions
    //private float[][] matrix = {{0.75f, 1.0f, 0.7f, 1.0f}, {0.8f, -0.95f, -0.85f, 0.95f}, {0.75f, 1.0f, -0.75f, -1.0f}};

    private float[][] matrix = DriveBaseVectors.arcadeDriveVectors;

    //Initializes with the hardwareMap
    @Override
    public void init() {

        left_front_drive =  hardwareMap.get(ExpansionHubMotor.class, "left_front_drive");
        left_back_drive =  hardwareMap.get(ExpansionHubMotor.class, "left_back_drive");
        right_front_drive =  hardwareMap.get(ExpansionHubMotor.class, "right_front_drive");
        right_back_drive =  hardwareMap.get(ExpansionHubMotor.class, "right_back_drive");

        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHub10 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 10");

        left_bottom_switch = hardwareMap.get(DigitalChannel.class, "left_bottom_switch");
        right_bottom_switch = hardwareMap.get(DigitalChannel.class, "right_bottom_switch");

        left_front_drive.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front_drive.setDirection(ExpansionHubMotor.Direction.REVERSE);
        right_back_drive.setDirection(ExpansionHubMotor.Direction.REVERSE);

        lift_left = hardwareMap.get(DcMotorSimple.class, "lift_left");
        lift_right = hardwareMap.get(DcMotorSimple.class, "lift_right");

        //Reverse the left side
        lift_left.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");



        left_bottom_switch.setMode(DigitalChannel.Mode.INPUT);
        right_bottom_switch.setMode(DigitalChannel.Mode.INPUT);




        driver = new Driver(gamepad1, hardwareMap);
        foundationMover = new FoundationMover(hardwareMap);
        foundationMover.init();


        foundationMover.down();




    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }


    /**
     * Current driver/manipulator controls
     *
     * Gamepad 1 = Driver(Parker)
     * Gamepad 2 - Manipulator(Jonah)
     *
     *
     * Gamepad 1 / left_stick = strafing
     * Gamepad 1 / right_stick = turning
     * Gamepad 1 / left_bumper = slow mode / normal activate
     * Gamepad 1 / right_bumper = reverse / normal activate
     * Gamepad 2 / dpad_up = lift up / lift hold toggle
     * Gamepad 2 / dpad_down = lift down / lift hold toggle
     * Gamepad 2 / left_bumper = claw open
     * Gamepad 2 / right_bumper = claw closed
     * Gamepad 2 / x = arm down
     * Gamepad 2 / y = arm up (slightly)
     * Gamepad 2 / right_stick = foundation movers
     *
     * Controls to be changed with the Config OpMode in future.
     *
     */
    @Override
    public void loop() {

        switch(new Random().nextInt(10)){
            case 0: expansionHub1.setLedColor(R.color.tomato);  break;
            case 1: expansionHub1.setLedColor(R.color.active_button_green); break;
            case 2: expansionHub1.setLedColor(R.color.darkorange); break;
            case 3: expansionHub1.setLedColor(R.color.aquamarine); break;
            case 4: expansionHub1.setLedColor(R.color.mintcream); break;
            case 5: expansionHub1.setLedColor(R.color.firebrick); break;
            case 6: expansionHub1.setLedColor(R.color.steelblue); break;
            case 7: expansionHub1.setLedColor(R.color.thistle); break;
            case 8: expansionHub1.setLedColor(R.color.turquoise); break;
            case 9: expansionHub1.setLedColor(R.color.lime); break;

        }


        //Get the data for this iteration
        revBulkData1 = expansionHub1.getBulkInputData();
        revBulkData10 = expansionHub10.getBulkInputData();




        /*
        Read from the limit switches and see if they are triggered
         */
        if(!revBulkData10.getDigitalInputState(right_bottom_switch) || !revBulkData1.getDigitalInputState(left_bottom_switch)){
            liftState = LiftState.AT_BOTTOM;
        }

        else {
            liftState = LiftState.MOVING;
        }

        //Move up
        if(gamepad2.dpad_up){
            lift_left.setPower(0.5);
            lift_right.setPower(0.5);

            liftState = LiftState.MOVING;
        }

        //Move down
        //The enum value is set by the limit switches
        else if(gamepad2.dpad_down){
            switch (liftState){
                case MOVING:
                    lift_left.setPower(-0.1);
                    lift_right.setPower(-0.1);

                    break;

                case AT_BOTTOM:
                    lift_left.setPower(0);
                    lift_right.setPower(0);

                    break;
            }
        }

        //Hold position
        else {
            lift_left.setPower(0.07);
            lift_right.setPower(0.07);
        }



        //Read driver inputs
        //inputs = new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x};


        //Calculate power for drive
        //outputs = m_v_mult(matrix, inputs);


        //Doesn't need to pass any parameters
        driver.drive();


        //Controls arm
        armToggle.update();


        //Same as above
        clawToggle.update();















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

    public interface ArcadeInput {
        float[] inputs();
    }


}