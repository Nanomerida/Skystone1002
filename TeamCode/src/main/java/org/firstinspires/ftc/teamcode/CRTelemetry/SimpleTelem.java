package org.firstinspires.ftc.teamcode.CRTelemetry;



import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/** A simple class for setting up telemetry */
public abstract class SimpleTelem {



    //A local telemetry object
    Telemetry telemetry;


    //Various aspects
    private String robotStatus;
    private String driveStatus;

    public SimpleTelem(Telemetry aTelemetry){
        this.telemetry = aTelemetry;
    }

}
