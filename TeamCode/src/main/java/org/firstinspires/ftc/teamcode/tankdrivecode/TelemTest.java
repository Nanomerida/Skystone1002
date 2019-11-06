package org.firstinspires.ftc.teamcode.tankdrivecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TelemTest extends LinearOpMode {
    TelemObjectTest thing = new TelemObjectTest(this);




    @Override
    public void runOpMode(){


        waitForStart();


        thing.showTelem("Hi:", "Nelitha!");
    }
}
