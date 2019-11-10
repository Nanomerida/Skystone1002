package org.firstinspires.ftc.teamcode.Methods;

public interface MecanumMovement {

    /**
     * Vector Position Change
     *
     * <p> This method calculates the motor powers needed to go from the current position to
     * a given position. Use with mecanum base. </p>
     *
     *
     * @param Xg Goal X position
     * @param Xa Actual X position
     * @param Yg Goal Y position
     * @param Ya Actual Y position
     * @return motorPower Motor powers in [LF, LB, RF, RB] format.
     */

    public double[] PositionChange(double Xg, double Xa, double Yg, double Ya);

    /**
     * Vector Angle Change
     *
     * <p> This method calculates the motor powers needed to go from the current heading to
     *    a given heading. Use with mecanum base.</p>
     * @param thetaG The heading goal.
     * @return motorPower The array of motor powers in [LF, LB, RF, RB] format.
     */

    public double[] AngleChange(double thetaG);

    /**
     * Goal Check for position
     *
     * <p> This method checks the current position with the position goal and sees if it
     * is close enough. </p>
     * @param Xa Actual X
     * @param Xg Goal X
     * @param Ya Actual Y
     * @param Yg Goal Y
     * @return reachedGoal True if goal reached
     */

    public boolean GoalCheckPos(double Xa, double Xg, double Ya, double Yg);

    public boolean GoalCheckAngle(double thetaG, double thetaA);

    public double degreeServoConv(double degrees);
}
