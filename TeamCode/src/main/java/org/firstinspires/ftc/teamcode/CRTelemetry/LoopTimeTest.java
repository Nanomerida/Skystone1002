package org.firstinspires.ftc.teamcode.CRTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Ping Test", group = "Testing")
public class LoopTimeTest extends OpMode {

    ElapsedTime ping = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



    @Override
    public void init(){

    }


    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        telemetry.addData("Loop Time milli", ping.milliseconds());
        telemetry.addData("Loop Time nano", ping.nanoseconds());

        ping.reset();

    }







}
