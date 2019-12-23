package org.firstinspires.ftc.teamcode.Mecanum.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

public class Driver {

    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;



    /**
     * Enum to represent the current speed mode of the drive base.
     */
    public enum DriveState {
        ULTRA_EPIC_FAST, //EPICALLY FAST!!!!!!!!!!!!!
        FAST, //slowwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
        FAST_REVERSE //slowwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww..........BUT REVERSE!
    }


    private Gamepad driver;
    public DriveState driveState = DriveState.ULTRA_EPIC_FAST;

    public boolean prevLeftBumper;
    public boolean prevRightBumper;




    public Driver(Gamepad driver, ArrayList<ExpansionHubMotor> driveMotors, boolean prevLeftBumper, boolean prevRightBumper){
        this.driver = driver;

        left_front_drive = driveMotors.get(0);
        left_back_drive = driveMotors.get(1);
        right_front_drive = driveMotors.get(2);
        right_back_drive = driveMotors.get(3);

        this.prevLeftBumper = prevLeftBumper;
        this.prevRightBumper = prevRightBumper;
    }

    public void drive(float[] outputs){
        if(driver.left_bumper && !prevLeftBumper){
            driveState = Driver.DriveState.FAST;
        }
        else if(driver.left_bumper && prevLeftBumper){
            driveState = Driver.DriveState.ULTRA_EPIC_FAST;
        }
        else if(driver.right_bumper && !prevRightBumper){
            driveState = Driver.DriveState.FAST_REVERSE;
        }
        else if(driver.right_bumper && prevRightBumper){
            driveState = Driver.DriveState.ULTRA_EPIC_FAST;
        }

        switch(driveState){
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
            case FAST_REVERSE:
                left_front_drive.setPower(outputs[0] * -0.5f);
                left_back_drive.setPower(outputs[1] * -0.5f);
                right_front_drive.setPower(outputs[2] * -0.5f);
                right_back_drive.setPower(outputs[3] * -0.5f);
        }

        left_front_drive.setPower(outputs[0]);
        left_back_drive.setPower(outputs[1]);
        right_front_drive.setPower(outputs[2]);
        right_back_drive.setPower(outputs[3]);

        //store current slow mode status
        prevLeftBumper = driver.left_bumper;
        prevRightBumper = driver.right_bumper;
    }



}
