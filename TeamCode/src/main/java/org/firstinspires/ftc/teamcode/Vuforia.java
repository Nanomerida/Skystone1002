/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//^ Blah, blah, blah, blah, ....

/* In the following code, I have edited things to be able to use this with the Skystone vuMark. This is originaly meant for
 * Relic Recovery (obviously), and some things still assumed the relic recovery trackables. I changed most of those to Skystone files,
 * like the data set used for loading VuforiaTrackables (if you are interested, it's in Skystone1002/FtcRobotController/src/main/assets,
 * under Skystone.xml. There are other Trackables there too for the pictures on the perimeter and the bridges, but the first one in the list is
 * the SkyStone vuMark.). I annotated certain things, which are usually the non-official looking comments. A few object names and
 * stuff like that I have changed to make more relevant to Skystone. As for using this, it just needs to return an int or something with
 * 0, 1, or 2. We will need to make all of this a method eventually, and make sure the same vuforia libraries are imported in our code.
 * I'm also going to set this up to be able to be a class called in the Autonomous file, and will comment out some things.
 * - Ryan Driemeyer */

package org.firstinspires.ftc.teamcode; //I think we needed to change this. Dunno though.

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/* import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix; //pretty sure we don't need all this positioning stuff, since it is only used
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;      // by the things I deleted from the sample (explained further down)
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation; */

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark; // <-- this is the enum explained below, there is no skystone equivalant in files
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId; //this is just vuforia stuff
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigationWebcam}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigationWebcam
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name="Vuforia", group ="Vuforia")
//@Disabled
public class Vuforia { //extends LinearOpMode


        /*NOTE. I am moving the main method for the Vuforia class up here, but I'm not sure if it will mess up some stuff.
         * Therefore, I will mark the original location of the method. */

        public static final String TAG = "Vuforia VuMark Sample";

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        VuforiaLocalizer vuforia;

        /**
         * This is the webcam we are to use. As with other hardware devices such as motors and
         * servos, this device is identified using the robot configuration tool in the FTC application.
         */
        WebcamName webcamName;
    public int visionTest() {      //changed name to visionTest, and made it a integer method.  This should NOT be a static method

        //ORIGINAL LOCATION OF METHOD

        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //no idea how this works

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */

        //We don't need this, unless we want a live view. I'll just comment it out for now.

        // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //This is what we'll use for now:

        // OR...  Do Not Activate the Camera Monitor View, to save power
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AeCl8dv/////AAABmU0vhtooEkbCoy9D8hM/5Yh8AhufXZT4nSVVD16Vjh1o/rLFmVyKVPNW3S/lXY0UWmDBpSNPS5yMk6lZoMFhTMoq9BMbmXHJ9KU+uKvC+GVp5cuEo18HvMpLMPPNmIVoXgOv9CqDfnRCOLSCblZf5cRF+E/LNqkZU7dEnEe/rrDq76FjVXruSdMBmUefIhu853VEpgvJPJTNopNjE0yU5TJ3+Uprgldx7fdy//VPG8PfXcaxLj4EJOzEKwJuCNdPS43bio37xbTbnLTzbKmfTqCI6BJpPaK5fXCk7o5xdVewJJbZCA8DDuNX6KUTT//OJ1UEWnMSYw5H1BrWMkytK5Syws7gdsCpYUshsQX7VP51";
        //vuforia key, I pasted in ours for now

        /**
         * We also indicate which camera on the RC we wish to use. For pedagogical purposes,
         * we use the same logic as in {@link ConceptVuforiaNavigationWebcam}.
         */
        parameters.cameraName = webcamName;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters); //I don't understand this yet, we will have to do this.

        //loads trackables. I will rename all of this to fit the skystone theme.

        VuforiaTrackables skystoneTrackables = this.vuforia.loadTrackablesFromAsset("Skystone"); //changed this correct Skystone file from Assets folder
        VuforiaTrackable skystoneStone = skystoneTrackables.get(0); //It uses relic template, I will just make this skystoneStone
        skystoneStone.setName("SKYSTONE"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start"); //useless telemetry >:(
        telemetry.update();

        skystoneTrackables.activate(); //activates the Vuforia, this is how we turn it on.
        boolean nofoundSkystone = true;
        int skystonePos = 0;

        while (nofoundSkystone) { // changed this condition since we are not using LinearOp, but this needs to loop until
            //a match is found. Maybe a boolean for the condition? I'll go with that for now.
            /**
             * See if any of the instances of {@link skystoneStone} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(skystoneStone); //I've found the enum in the FTC Library, but it's pretty specific.
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {                          //So for now, the name will have to stay the same. :(

                /* This will be triggered when the status of the vuMark detected is anything other than UNKNOWN.
                This is where we will need to put all the stuff we want to get from this code. The object "vuMark" is a constant
                from the above enum which is either "UNKNOWN, LEFT, RIGHT, or CENTER". My recommendation is that we have a switch
                statement with the conditions based on the position of the vuMark.*/
                if(vuMark == RelicRecoveryVuMark.LEFT) {
                    skystonePos = 0;
                    nofoundSkystone = false;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    skystonePos = 1;
                    nofoundSkystone = false;
                } else (vuMark == RelicRecoveryVuMark.RIGHT) {
                    skystonePos = 2;
                    nofoundSkystone = false;
                }

            }

            /* As far as I can tell, this is as much as we need for what we are doing with just finding the Skystone. The rest is just
             * doing other things with the vuMark, like localization. We don't really need that, so I will delete most of it. It can be
             * found in the FTC sample code if desired. -RyanD */


            else {
                //I'll leave this else statement though. Who knows? ( ͡° ͜ʖ ͡°)
            }
        }
        return skystonePos;
    }
}
