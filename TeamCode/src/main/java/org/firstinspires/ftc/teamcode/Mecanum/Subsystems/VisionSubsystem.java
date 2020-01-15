package org.firstinspires.ftc.teamcode.Mecanum.Subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CRVuforia.Vuforia;

public class VisionSubsystem implements Subsystem {

    Vuforia vision;

    public VisionSubsystem(HardwareMap hardwareMap){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    @Override
    public void initHardware(){


    }

    @Override
    public void periodic(){


    }
}
