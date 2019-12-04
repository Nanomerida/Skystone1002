package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name= "ff", group = "f")
public class jkfhkasfj extends LinearOpMode {



    ExpansionHubMotor ly;
    ExpansionHubMotor ry;
    ExpansionHubMotor x;
    RevBulkData g;
    ExpansionHubEx t;


    @Override
    public void runOpMode(){



        ly = (ExpansionHubMotor)hardwareMap.get(DcMotor.class, "ly");
        ry = (ExpansionHubMotor)hardwareMap.get(DcMotor.class, "ry");
        x = (ExpansionHubMotor)hardwareMap.get(DcMotor.class, "x");
        t = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        ry.setDirection(ExpansionHubMotor.Direction.REVERSE);

        ly.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        ry.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        x.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);

        ly.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        ly.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        ly.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            g = t.getBulkInputData();
            telemetry.addData("Count", g.getMotorCurrentPosition(ly));
            telemetry.addData("Count", g.getMotorCurrentPosition(ry));
            telemetry.addData("Count", g.getMotorCurrentPosition(x));
            telemetry.update();

            if(gamepad1.x){
                ly.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
                ry.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
                x.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }


    }
}
