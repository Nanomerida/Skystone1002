package org.firstinspires.ftc.teamcode.Variables;

import com.qualcomm.robotcore.hardware.HardwareDeviceHealth.HealthStatus;


public class RobotState {

    public enum MainState {
        IDLE,
        ACTIVE
    }

    public enum DriveState {
        IDLE,
        MOVING
    }

    public enum VisionState {
        DISABLED,
        SEARCHING,
        FOUND,
        ERROR;
    }

    public enum GyroState {
        INITIALIZING,
        ACTIVE
    }

    public enum IntakeState {
        IDLE,
        ACTIVE
    }

    public enum LiftState {
        IDLE,
        MOVING_UP,
        MOVING_DOWN
    }


    public static MainState mainState;
    public static DriveState driveState;
    public static VisionState visionState;
    public static GyroState gyroState;
    public static IntakeState intakeState;
    public static LiftState liftState;


    public DriveState getDriveState() {
        return driveState;
    }

    public void setDriveState(DriveState newState) {
        driveState = newState;
    }

    public MainState getMainState() {
        return mainState;
    }

    public void setMainState(MainState newState){
        mainState = newState;
    }

    public VisionState getVisionState(){
        return visionState;
    }

    public void setVisionState(VisionState newState) {
        visionState = newState;
    }

    public IntakeState getIntakeState(){
        return intakeState;
    }

    public void setIntakeState(IntakeState newState){
        intakeState = newState;
    }

    public LiftState getLiftState(){
        return liftState;
    }

    public void setLiftState(LiftState newState){
        liftState = newState;
    }
}
