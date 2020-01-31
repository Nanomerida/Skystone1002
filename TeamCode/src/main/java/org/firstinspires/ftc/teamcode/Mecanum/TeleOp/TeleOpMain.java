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

import org.firstinspires.ftc.robotcore.external.Func;
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

    private Driver driver;
    private FoundationMover foundationMover;




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



    private FoundationState foundationMoverState = FoundationState.LOCKED_ON_FOUNDATION;


/**Funny Number*/



    /**
     * Toggle control the claw servo.
     */
    private Toggle clawToggle = () -> {

        /*if(gamepad2.left_bumper) {
            if (clawState == ClawState.OPEN) {
                claw.setPosition(0);
                clawState = ClawState.CLOSED;
            } else if (clawState == ClawState.CLOSED) {
                claw.setPosition(0.4);
                clawState = ClawState.OPEN;
            }
        } */

        if(gamepad2.left_bumper){
            claw.setPosition(0.05);
        }
        else if(gamepad2.right_bumper){
            claw.setPosition(0.4);
        }

    };

    private Toggle foundationToggle = () -> {

        if(gamepad2.x){
            if(foundationMoverState == FoundationState.FULLY_UP) {
                foundationMover.close();
                foundationMoverState = FoundationState.LOCKED_ON_FOUNDATION;
            }
            else if(foundationMoverState == FoundationState.LOCKED_ON_FOUNDATION){
                foundationMover.up();
                foundationMoverState = FoundationState.FULLY_UP;
            }
        }
    };

   private Toggle armToggle = () -> {
        if(gamepad2.a){
            arm.setPosition(1);
        }
        else if(gamepad2.b){
            arm.setPosition(0.25);
        }
    };

    private Func<Boolean> limitSwitches = () -> {
        return !revBulkData1.getDigitalInputState(left_bottom_switch) || !revBulkData10.getDigitalInputState(right_bottom_switch);
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
        //arm.setPosition(1);

    }



    @Override
    public void loop() {



        //Get the data for this iteration
        revBulkData1 = expansionHub1.getBulkInputData();
        revBulkData10 = expansionHub10.getBulkInputData();




        /*
        Read from the limit switches and see if they are triggered
         */

        //Move up
        if(gamepad2.dpad_up){
            lift_left.setPower(0.5);
            lift_right.setPower(0.5);

        }



        //Move down
        //The enum value is set by the limit switches
        else if(gamepad2.dpad_down && !limitSwitches.value()){
            lift_left.setPower(-0.08);
            lift_right.setPower(-0.08);
        }

        //Hold position
        else if(!limitSwitches.value()) {
            lift_left.setPower(0.09);
            lift_right.setPower(0.09);
        }
        else {
            lift_left.setPower(0);
            lift_right.setPower(0);
        }



        //Read driver inputs
        //inputs = new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x};


        //Calculate power for drive
        //outputs = m_v_mult(matrix, inputs);

        float[] input = {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x};
        float[] output = m_v_mult(matrix, input);

        driver.drive(output);



        //Controls arm
        armToggle.update();


        //Same as above
        clawToggle.update();





        foundationToggle.update();




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