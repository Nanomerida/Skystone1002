package org.firstinspires.ftc.teamcode.Mecanum;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth;
import org.firstinspires.ftc.robotcore.external.Func;

public class ShowTelemetry {

    private Telemetry telemetry; //local telemetry

    public ShowTelemetry(Telemetry telem){
        this.telemetry = telem;
    } //constructor


    //value Strings
    public String driveStatusV = "d";
    public String visionStatusV = "d";
    public int stepNumbV = 0;
    public double currentHeadingV = 0;
    public String robotHealthV = "d";







    public void startTelemetry(){

        telemetry.addData("Drive Base Status:", new Func<String>(){
            @Override public String value() {
                return driveStatusV;
            }
        });

        telemetry.addData("Vision Testing Status:", new Func<String>(){
            @Override public String value(){
                return visionStatusV;
            }
        });

        telemetry.addData("Current Step Number:", new Func<Integer>(){
            @Override
            public Integer value() {
                return stepNumbV;
            }
        });

        telemetry.addData("Current Heading:", new Func<Double>(){
            @Override
            public Double value() {
                return currentHeadingV;
            }
        });

        telemetry.addData("Robot Health", new Func<String>(){
            @Override public String value() {
                return robotHealthV;
            }
        });


        telemetry.update();
    }

    public void massTelemetryUpdate(String drive, String vision, int step, double heading, String health){
        setDriveStatus(drive);
        setVisionStatus(vision);
        setStepNumbV(step);
        setCurrentHeadingV(heading);
        setRobotHealth(health);
        updateTelemetry();
    }

    public void updateTelemetry(){
        telemetry.update();
    }


    public void setDriveStatus(String status){
        driveStatusV = status;
    }


    public void  setVisionStatus(String status){
        visionStatusV = status;
    }

    public void setRobotHealth(String status){
        robotHealthV = status;
    }

    public void setStepNumbV(int step){
        stepNumbV = step;
    }

    public void setCurrentHeadingV(double heading){
        currentHeadingV = heading;
    }

    public void telemetryMessage(String caption, String value){telemetry.addData(caption, value);}

    public void telemetryMessage(String caption, double value) { telemetry.addData(caption, value);}








}
