package org.firstinspires.ftc.teamcode.Mecanum.Config;


/**
 * Class to hold the manipulator controls
 */
public class ManipulatorControls {

    public DriverConfig.DriverName name = DriverConfig.DriverName.JONAH;

    public DriverConfig.Button lift_up = DriverConfig.Button.DPAD_UP;
    public DriverConfig.Button lift_down = DriverConfig.Button.DPAD_DOWN;
    public DriverConfig.Button claw_open = DriverConfig.Button.LEFT_BUMPER;
    public DriverConfig.Button claw_closed = DriverConfig.Button.RIGHT_BUMPER;
    public DriverConfig.Button arm_down = DriverConfig.Button.X;
    public DriverConfig.Button arm_up = DriverConfig.Button.Y;
    /**
     * Make sure to reverse this!
     */
    public DriverConfig.Trigger foundation_movers = DriverConfig.Trigger.RIGHT_STICK_Y;

}
