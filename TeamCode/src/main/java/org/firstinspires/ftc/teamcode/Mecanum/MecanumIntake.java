package org.firstinspires.ftc.teamcode.Mecanum;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.Timer;


/**Class for the current mecanum intake as of 12/19/19. This uses a servo arm, a servo class, and a two motor lift with
 * limit switches.
 *
 * TODO: Need to set arm positions
 *
 */

public class MecanumIntake {

    LinearOpMode opMode;

    Timer timer;


    private DcMotorSimple lift_left;
    private DcMotorSimple lift_right;

    private Servo arm;
    private Servo claw;

    private DigitalChannel left_top_switch;
    private DigitalChannel left_bottom_switch;
    private DigitalChannel right_top_switch;
    private DigitalChannel right_bottom_switch;

    private static double clawOpen = 0;
    private static double clawClosed = 0.4;
    private static double armDown = 1;
    private static double armUpSlight = 0.9;
    private static double liftUpPower = 0.5;
    private static double liftDownPower = -0.1;
    private static double liftHoldPower = 0.05;

    public MecanumIntake(LinearOpMode opMode, HardwareMap hardwareMap){

        this.opMode = opMode;
        timer = new Timer();

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
    public MecanumIntake(HardwareMap hardwareMap){
        timer = new Timer();

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

    public void initIntake(){
        armDown();
        openClaw();

    }

    /**Are the bottom limit switches triggered? */
    public boolean liftAtBottom(){
        return (left_bottom_switch.getState() || right_bottom_switch.getState());
    }

    /**Are the top limit switches triggered? */
    public boolean liftAtTop(){
        return (left_top_switch.getState() || right_top_switch.getState());
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

    /** We don't have encoders, so this must be in time. It will
     * go up until the time runs out or it reaches the top.
     * @param millis time to go up for.
     */
    public void liftUpForTime(long millis){
        timer.setTimer(millis);
        timer.startTimer();
        while(opMode.opModeIsActive() && !timer.timerDone() && liftAtTop()){
            moveLiftUp();
        }
    }

    /** We don't have encoders, so this must be in time. It will
     * go down until the time runs out or it reaches the bottom.
     * @param millis time to go down for.
     */
    public void liftDownForTime(long millis){
        timer.setTimer(millis);
        timer.startTimer();
        while(opMode.opModeIsActive() && !timer.timerDone() && liftAtTop()){
            moveLiftDown();
        }
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
        if(opMode.opModeIsActive() && !liftAtBottom()){
            lift_left.setPower(liftDownPower);
            lift_right.setPower(liftDownPower);
        }
    }

    /** Hold the lift at the current position. */
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


}
