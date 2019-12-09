package org.firstinspires.ftc.teamcode.hardwareMaps;



import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Duo motor rtp test", group = "Testing")
@Disabled
public class ImuUtil extends OpMode {

    private DcMotor lift = null;

    private int counts;


    @Override
    public void init() {

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        counts = lift.getCurrentPosition();
        telemetry.addData("Counts:", counts);

        if (gamepad1.dpad_up) {
            telemetry.addData("Direction:", "up");
            lift.setPower(.5);
        } else if (gamepad1.dpad_down) {
            telemetry.addData("Direction:", "down");
            lift.setPower(-.5);
        } else {
            telemetry.addData("Direction:", "none");
            lift.setPower(0);
        }
        telemetry.update();

    }

    @Override
    public void stop() {
        lift.setPower(0);
    }

 }
    
    
      
