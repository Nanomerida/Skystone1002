package org.firstinspires.ftc.teamcode.Mecanum.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BulkDataManager;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;


/**Class for the current mecanum intake as of 12/19/19. This uses a servo arm, a servo class, and a two motor lift with
 * limit switches.
 *
 *
 *
 */

public class MecanumIntake implements Subsystem {

    public BulkDataManager bulkDataManager;
    public RevBulkData bulkData10;


    public DcMotorSimple lift_left = null;
    public DcMotorSimple lift_right = null;

    public Servo arm = null;
    public Servo claw = null;

    public DigitalChannel left_top_switch = null;
    public DigitalChannel left_bottom_switch = null;
    public DigitalChannel right_top_switch = null;
    public DigitalChannel right_bottom_switch = null;

    private static double clawOpen = 0;
    private static double clawClosed = 0.4;
    private static double armDown = 1;
    private static double armUpSlight = 0.9;
    private static double liftUpPower = 0.5;
    private static double liftDownPower = -0.1;
    private static double liftHoldPower = 0.05;

    public MecanumIntake(HardwareMap hardwareMap) {


        bulkDataManager = new BulkDataManager(hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 10"), bulkData10);


        lift_left = hardwareMap.get(DcMotorSimple.class, "lift_left");
        lift_right = hardwareMap.get(DcMotorSimple.class, "lift_right");

        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");

        left_top_switch = hardwareMap.get(DigitalChannel.class, "left_top_switch");
        left_bottom_switch = hardwareMap.get(DigitalChannel.class, "left_bottom_switch");
        right_top_switch = hardwareMap.get(DigitalChannel.class, "right_top_switch");
        right_bottom_switch = hardwareMap.get(DigitalChannel.class, "right_bottom_switch");

        left_top_switch.setMode(DigitalChannel.Mode.INPUT);
        left_bottom_switch.setMode(DigitalChannel.Mode.INPUT);
        right_top_switch.setMode(DigitalChannel.Mode.INPUT);
        right_bottom_switch.setMode(DigitalChannel.Mode.INPUT);

        lift_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init(){
        armDown();
        openClaw();

    }

    /**Are the bottom limit switches triggered? */
    public boolean liftAtBottom(){
        bulkDataManager.refreshBulkData();
        return (!bulkData10.getDigitalInputState(left_bottom_switch) || !bulkData10.getDigitalInputState(right_bottom_switch));
    }

    /**Are the top limit switches triggered? */
    public boolean liftAtTop(){
        bulkDataManager.refreshBulkData();
        return (!bulkData10.getDigitalInputState(left_top_switch) || !bulkData10.getDigitalInputState(right_top_switch));
    }

    /**Opens the claw. */
    public void openClaw(){
        claw.setPosition(clawOpen);
    }

    /** Closes the claw. */
    public void closeClaw(){
        claw.setPosition(clawClosed);
    }

    /** Moves arm up a little. Not recommended. */
    public void armUp(){
        arm.setPosition(armUpSlight);
    }

    /**Moves arm down to horizontal pose */
    public void armDown(){
        arm.setPosition(armDown);
    }

    /**Move lift up */
    public void moveLiftUp(){
        if(!liftAtTop()){
            lift_left.setPower(liftUpPower);
            lift_right.setPower(liftUpPower);
        }
    }

    /** Move lift down */
    public void moveLiftDown(){
        if(!liftAtBottom()){
            lift_left.setPower(liftDownPower);
            lift_right.setPower(liftDownPower);
        }
    }

    /** Hold the lift at the current position. If it's at the bottom, then just stop */
    public void holdLift(){
        if(!liftAtBottom()){
            lift_left.setPower(liftHoldPower);
            lift_right.setPower(liftHoldPower);
        }
        else {
            stopLift();
        }
    }

    /**Set zero power to motors. */
    public void stopLift(){
        lift_left.setPower(0);
        lift_right.setPower(0);
    }

    public Function0<Unit> getClawClose(){
        return () -> {
            closeClaw();
            return Unit.INSTANCE;
        };
    }

    public Function0<Unit> getClawOpen(){
        return () -> {
            openClaw();
            return Unit.INSTANCE;
        };
    }



}
