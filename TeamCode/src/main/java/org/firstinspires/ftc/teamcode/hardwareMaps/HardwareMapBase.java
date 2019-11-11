package org.firstinspires.ftc.teamcode.hardwareMaps;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public abstract class HardwareMapBase {


    private ArrayList<DcMotor>  driveMotors;
    private ArrayList<CRServo> servos;
    private ArrayList<DcMotor> robotMotors;
    private BNO055IMU imu;


    public HardwareMapBase(ArrayList<DcMotor> aDriveMotors, ArrayList<CRServo> aServo, ArrayList<DcMotor> aRobotMotors, BNO055IMU aImu){
        this.driveMotors.addAll(aDriveMotors);
        this.servos.addAll(aServo);
        this.robotMotors.addAll(aRobotMotors);
        this.imu = aImu;
    }


    protected ArrayList<DcMotor> getDriveMotors() {
        return driveMotors;
    }

    protected ArrayList<CRServo> getServos() {
        return servos;
    }

    protected ArrayList<DcMotor> getRobotMotors() {
        return robotMotors;
    }

    protected BNO055IMU getImu() {
        return imu;
    }

    public abstract void setSensors();
    public abstract void init(OpMode aOpMode);




}
