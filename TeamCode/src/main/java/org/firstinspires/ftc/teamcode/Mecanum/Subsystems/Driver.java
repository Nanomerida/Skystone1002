package org.firstinspires.ftc.teamcode.Mecanum.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mecanum.TeleOp.TeleOpMain;
import org.firstinspires.ftc.teamcode.hardware.DriveBaseVectors;
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
    private DriveState driveState = DriveState.ULTRA_EPIC_FAST;

    TeleOpMain.ArcadeInput arcadeInput;

    private float[] inputs;





    public Driver(Gamepad driver, HardwareMap hardwareMap){
        this.driver = driver;

        left_front_drive =  hardwareMap.get(ExpansionHubMotor.class, "left_front_drive");
        left_back_drive =  hardwareMap.get(ExpansionHubMotor.class, "left_back_drive");
        right_front_drive =  hardwareMap.get(ExpansionHubMotor.class, "right_front_drive");
        right_back_drive =  hardwareMap.get(ExpansionHubMotor.class, "right_back_drive");

        arcadeInput = () -> new float[] {this.driver.left_stick_y, this.driver.left_stick_x, -this.driver.right_stick_x};


    }

    public void update(){

        if(driver.left_bumper){
            driveState = DriveState.FAST;
        }
        else if(driver.right_bumper){
            driveState = DriveState.FAST_REVERSE;
        }
        else{
            driveState = DriveState.ULTRA_EPIC_FAST;
        }
    }

    public void drive(float[] outputs){


        if(driver.left_bumper){
            driveState = DriveState.FAST;
        }
        else if(driver.right_bumper){
            driveState = DriveState.FAST_REVERSE;
        }
        else{
            driveState = DriveState.ULTRA_EPIC_FAST;
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

    }
    public void drive(){
        float[] outputs = m_v_mult(DriveBaseVectors.arcadeDriveVectors, new float[] {this.driver.left_stick_y, this.driver.left_stick_x, -this.driver.right_stick_x});

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
    }


    private static float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }



}