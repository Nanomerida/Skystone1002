package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WebcamInit {

    public WebcamName webcamName;
    HardwareMap hwMap           =  null;

    //constructor
    public WebcamInit(){

    }
    public void init(HardwareMap ahwMpa){
        hwMap = ahwMpa;

        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

    }

}
