package org.firstinspires.ftc.teamcode.hardwareMaps;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** This is not an OpMode for anything.
 * This is just a class that contains the declaration/initialization information for the web-cam on vuforia.
 */

public class HardwareMapWebcam {

    public WebcamName webcamName;
    HardwareMap hwMap           =  null;

    //constructor
    public HardwareMapWebcam(){

    }
    public void init(HardwareMap ahwMpa){
        hwMap = ahwMpa;

        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

    }

}
