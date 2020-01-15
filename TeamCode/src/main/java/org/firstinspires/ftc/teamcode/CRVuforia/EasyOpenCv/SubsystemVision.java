package org.firstinspires.ftc.teamcode.CRVuforia.EasyOpenCv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;

public class SubsystemVision {

    public Init3BlockDetection pipeline;

    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private OpenCvCamera camera;

    public SubsystemVision(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.hardwareMap = hardwareMap;

        this.opMode = opMode;

    }

    /**
     * Initiates all the necessary hardware.
     */
    public void initHardware(){

        pipeline = new NaiveRectangleSamplingSkystoneDetectionPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(pipeline);

    }

    /**Starts the stream. Must be called after {@link SubsystemVision#initHardware()}.
     *
     */
    public void startVision(){
        camera.startStreaming(pipeline.getWidth(), pipeline.getHeight(), OpenCvCameraRotation.UPRIGHT);
    }

    @Deprecated
    public void onInit() {
        initHardware();
        camera.startStreaming(pipeline.getWidth(), pipeline.getHeight(), OpenCvCameraRotation.UPRIGHT);
    }




    /**
     * Telemetry to use for the stream. Intended for the duration of the streaming
     */
    public void streamLoop() {
        opMode.telemetry.addData("Skystone", () -> pipeline.getDetectedSkystonePosition());
        opMode.telemetry.addData("Skystone Positions",
                () -> pipeline.getSkystonePositions(3)[0] + "" + pipeline.getSkystonePositions(3)[1]);
        opMode.telemetry.addData("Frame Count", () -> camera.getFrameCount());
        opMode.telemetry.addData("FPS", () -> String.format(Locale.US,"%.2f", camera.getFps()));
        opMode.telemetry.addData("Total frame time ms", () ->  camera.getTotalFrameTimeMs());
        opMode.telemetry.addData("Pipeline time ms", () -> camera.getPipelineTimeMs());
        opMode.telemetry.addData("Overhead time ms", () -> camera.getOverheadTimeMs());
        opMode.telemetry.addData("Theoretical max FPS", () -> camera.getCurrentPipelineMaxFps());

        opMode.telemetry.update();
        opMode.sleep(10);
    }


    public void stopVision() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}