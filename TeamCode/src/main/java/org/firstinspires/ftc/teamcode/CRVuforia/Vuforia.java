
package org.firstinspires.ftc.teamcode.CRVuforia;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public class Vuforia {

    /**
     * Enum to represent the position of the skystone
     *
     * TODO Replace usages of an int to represent the position with this.
     */
    public enum SkystonePositon {
        LEFT,
        RIGHT,
        UNSEEN,
        UNKNOWN
    }

    private ElapsedTime searchTime = new ElapsedTime(0);

    /* vuforia key */
    private static final String VUFORIA_KEY =
            "AeCl8dv/////AAABmU0vhtooEkbCoy9D8hM/5Yh8AhufXZT4nSVVD16Vjh1o/rLFmVyKVPNW3S/lXY0UWmDBpSNPS5yMk6lZoMFhTMoq9BMbmXHJ9KU+uKvC+GVp5cuEo18HvMpLMPPNmIVoXgOv9CqDfnRCOLSCblZf5cRF+E/LNqkZU7dEnEe/rrDq76FjVXruSdMBmUefIhu853VEpgvJPJTNopNjE0yU5TJ3+Uprgldx7fdy//VPG8PfXcaxLj4EJOzEKwJuCNdPS43bio37xbTbnLTzbKmfTqCI6BJpPaK5fXCk7o5xdVewJJbZCA8DDuNX6KUTT//OJ1UEWnMSYw5H1BrWMkytK5Syws7gdsCpYUshsQX7VP51";


    // Class Members
    private OpenGLMatrix skystonePositionCoords = null;
    public VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */



    private int skystonePosition = 4;

    private WebcamName webcamName = null;


    /**
     * This is the opMode using this.
     */
    private LinearOpMode opMode;


    public SkystonePositon stonePosition = SkystonePositon.UNKNOWN;

    public VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
    public VuforiaTrackable stoneTarget = targetsSkyStone.get(0);




    /** This method initiates vuforia. Make sure to pass a webcam object
     *
     * @param awebcamName The webcam object
     * */
    public void blueInit(WebcamName awebcamName, LinearOpMode opMode) {

        this.webcamName = awebcamName;

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;


        /*
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }


    /**The method that looks for the Skystone for 5 seconds
     *
     * @return An enum value holding the results.
     */
    public SkystonePositon testVision() {

        stoneTarget.setName("Skystone");




        targetsSkyStone.activate();
        searchTime.reset();
        while (searchTime.seconds() <= 5 && opMode.opModeIsActive()) {

            // check all the trackable targets to see which one (if any) is visible.
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                skystonePositionCoords = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getVuforiaCameraFromTarget(); //give pose of trackable, returns null if not visible
                VectorF skystoneCoords = skystonePositionCoords.getTranslation();
                float closestX = Range.clip(skystoneCoords.get(0), -10f, 10f);
                if (closestX == -10) stonePosition = SkystonePositon.LEFT;
                else stonePosition = SkystonePositon.RIGHT;
                break;
            }
        }
        if(stonePosition != SkystonePositon.LEFT && stonePosition != SkystonePositon.RIGHT) stonePosition = SkystonePositon.UNSEEN;
       


        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
        return stonePosition;

    }

    /**
     * This is the old deprecated method, but it is here because it is used everywhere in earlier code.
     * @return int with position
     */
    @Deprecated
    public int visionTest() {


        stoneTarget.setName("Skystone");




        targetsSkyStone.activate();
        searchTime.reset();
        while (searchTime.seconds() <= 5 && opMode.opModeIsActive()) {

            // check all the trackable targets to see which one (if any) is visible.
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                skystonePositionCoords = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getVuforiaCameraFromTarget(); //give pose of trackable, returns null if not visible
                VectorF skystoneCoords = skystonePositionCoords.getTranslation();
                float closestX = Range.clip(skystoneCoords.get(0), -10f, 10f);
                if (closestX == -10) stonePosition = SkystonePositon.LEFT;
                else stonePosition = SkystonePositon.RIGHT;
                break;
            }
        }
        if(stonePosition != SkystonePositon.LEFT && stonePosition != SkystonePositon.RIGHT) stonePosition = SkystonePositon.UNSEEN;



        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
        return skystonePosition;

    }
}
