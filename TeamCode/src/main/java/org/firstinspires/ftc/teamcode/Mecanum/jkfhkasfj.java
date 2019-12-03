package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name= "ff", group = "f")
public class jkfhkasfj extends LinearOpMode {



    ExpansionHubMotor f;
    RevBulkData g;
    ExpansionHubEx t;


    @Override
    public void runOpMode(){



        f = (ExpansionHubMotor)hardwareMap.get(DcMotor.class, "f");
        t = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        waitForStart();

        while (opModeIsActive()){
            g = t.getBulkInputData();
            telemetry.addData("Count", g.getMotorCurrentPosition(f));
            telemetry.update();
        }


    }
}
