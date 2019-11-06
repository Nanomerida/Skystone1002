package org.firstinspires.ftc.teamcode.tankdrivecode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TelemObjectTest {


    OpMode opMode;

    public TelemObjectTest(OpMode aopMode){
        this.opMode = aopMode;
    }


    public void showTelem(String caption, String message){
        opMode.telemetry.addData(caption, message);
        opMode.telemetry.update();
    }


}
