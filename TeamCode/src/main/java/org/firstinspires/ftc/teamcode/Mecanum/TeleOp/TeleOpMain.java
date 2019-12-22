package org.firstinspires.ftc.teamcode.Mecanum.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.*;

import java.util.ArrayList;

@TeleOp(name = "TeleOpMain", group="TeleOp")

public class TeleOpMain extends OpMode {

    TeleOpFieldCentric fieldCentric;
    Driver driver;
    ElapsedTime ping = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


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

    private DigitalChannel left_top_switch = null;
    private DigitalChannel left_bottom_switch = null;
    private DigitalChannel right_top_switch = null;
    private DigitalChannel right_bottom_switch = null;


    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;


    float[] inputs;
    float[] outputs;

    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<>();


    private Driver.DriveState driveState = Driver.DriveState.ULTRA_EPIC_FAST;

    public static boolean opModeIsDone = false;



    private static float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }


    
    //Array to hold movement instructions
    private float[][] matrix = {{0.75f, 1.0f, 0.7f, 1.0f}, {0.8f, -0.95f, -0.85f, 0.95f}, {0.75f, 1.0f, -0.75f, -1.0f}};

    //Initializes with the hardwareMap
    @Override
    public void init() {

        left_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "left_front_drive");
        left_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "left_back_drive");
        right_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "right_front_drive");
        right_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "right_back_drive");

        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHub10 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 10");

        /*left_top_switch = hardwareMap.get(DigitalChannel.class, "left_top_switch");
        left_bottom_switch = hardwareMap.get(DigitalChannel.class, "left_bottom_switch");
        right_top_switch = hardwareMap.get(DigitalChannel.class, "right_top_switch");
        right_bottom_switch = hardwareMap.get(DigitalChannel.class, "right_bottom_switch"); */

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);

        lift_left = hardwareMap.get(DcMotorSimple.class, "lift_left");
        lift_right = hardwareMap.get(DcMotorSimple.class, "lift_right");

        //Reverse the left side
        lift_right.setDirection(DcMotorSimple.Direction.REVERSE);

        /*left_top_switch.setMode(DigitalChannel.Mode.INPUT);
        left_bottom_switch.setMode(DigitalChannel.Mode.INPUT);
        right_top_switch.setMode(DigitalChannel.Mode.INPUT);
        right_bottom_switch.setMode(DigitalChannel.Mode.INPUT); */


        driveMotors.add(left_front_drive);
        driveMotors.add(left_back_drive);
        driveMotors.add(right_front_drive);
        driveMotors.add(right_back_drive);


        driver = new Driver(gamepad1, driveMotors);
        //MecanumIntake intake = new MecanumIntake(hardwareMap, opModeIsDone);
        //intake.init();



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
        revBulkData10 = expansionHub10.getBulkInputData();

        //Reading the limit switches.
        /*limitSwitchStates.set(0, !revBulkData1.getDigitalInputState(left_bottom_switch));
        limitSwitchStates.set(1, !revBulkData1.getDigitalInputState(right_bottom_switch));
        limitSwitchStates.set(2, !revBulkData1.getDigitalInputState(left_top_switch));
        limitSwitchStates.set(3, !revBulkData1.getDigitalInputState(right_top_switch)); */


        //Move lift according to driver input
            if(gamepad2.dpad_up) {
                //intake.moveLiftUp();
                lift_left.setPower(0.5);
                lift_right.setPower(0.5);
            }
            else if(gamepad2.dpad_down) {
                //intake.moveLiftDown();
                lift_left.setPower(-0.1);
                lift_right.setPower(-0.1);
            }
            else {
                //intake.holdLift();
                lift_left.setPower(0.05);
                lift_right.setPower(0.05);
            }




        //Turn slow mode off, if pressed and not already active
        //driveState = (gamepad1.left_bumper && !prevLeftBumper) ? Driver.DriveState.FAST : Driver.DriveState.ULTRA_EPIC_FAST;

        //Read driver inputs
        //inputs = new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x};



        //Calculate power for drive
        //outputs = m_v_mult(matrix, inputs);

        driver.drive(m_v_mult(matrix, new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x}));




        /*//Move drive base
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

         */

        //Move lift
        /*if(gamepad2.dpad_up) {lift_left.setPower(-0.5); lift_right.setPower(-0.5); }
        else if(gamepad2.dpad_down) {lift_left.setPower(0.1); lift_right.setPower(0.1); }
        else lift_left.setPower(0); lift_right.setPower(0); */


        
        //Control claw
        if(gamepad2.right_bumper) claw.setPosition(0);
        else if(gamepad2.left_bumper) claw.setPosition(0.4);



        //Move arm
        if(gamepad2.a) arm.setPosition(1);
        else if(gamepad2.b) arm.setPosition(0.9);

        /*if(gamepad2.a) intake.armDown();
        else if(gamepad2.b) intake.armUp(); */

        //store current slow mode status
        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;




        //Useful telemetry
        telemetry.addLine("Motor Velocities: ");
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
        //intake.stopLift();
    }
}
