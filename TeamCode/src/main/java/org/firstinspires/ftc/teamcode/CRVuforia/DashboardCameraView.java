package org.firstinspires.ftc.teamcode.CRVuforia;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class DashboardCameraView extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AeCl8dv/////AAABmU0vhtooEkbCoy9D8hM/5Yh8AhufXZT4nSVVD16Vjh1o/rLFmVyKVPNW3S/lXY0UWmDBpSNPS5yMk6lZoMFhTMoq9BMbmXHJ9KU+uKvC+GVp5cuEo18HvMpLMPPNmIVoXgOv9CqDfnRCOLSCblZf5cRF+E/LNqkZU7dEnEe/rrDq76FjVXruSdMBmUefIhu853VEpgvJPJTNopNjE0yU5TJ3+Uprgldx7fdy//VPG8PfXcaxLj4EJOzEKwJuCNdPS43bio37xbTbnLTzbKmfTqCI6BJpPaK5fXCk7o5xdVewJJbZCA8DDuNX6KUTT//OJ1UEWnMSYw5H1BrWMkytK5Syws7gdsCpYUshsQX7VP51";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}