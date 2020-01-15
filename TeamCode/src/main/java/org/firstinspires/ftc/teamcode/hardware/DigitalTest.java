package org.firstinspires.ftc.teamcode.hardware;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class DigitalTest  extends LinearOpMode {


    private ExpansionHubEx hub1;
    private ExpansionHubEx hub10;
    private RevBulkData bulkData1;
    private RevBulkData bulkData10;
    private DigitalChannel left_switch;
    private DigitalChannel right_switch;

    private Telemetry.Item left_switch_value;
    private Telemetry.Item right_switch_value;


    @Override
    public void runOpMode(){
        hub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        hub10 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 10");

        left_switch = hardwareMap.get(DigitalChannel.class, "left_bottom_switch");
        right_switch = hardwareMap.get(DigitalChannel.class, "right_bottom_switch");

        left_switch.setMode(DigitalChannel.Mode.INPUT);
        right_switch.setMode(DigitalChannel.Mode.INPUT);



        waitForStart();

        while(opModeIsActive()){

            telemetry.update();
        }
    }

}
