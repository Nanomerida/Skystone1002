package org.firstinspires.ftc.teamcode.CRVuforia.EasyOpenCv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvPipeline#onViewportTapped()}
 * callback to switch which stage of a pipeline is rendered to the viewport for debugging
 * purposes. We also show how to get data from the pipeline to your OpMode.
 */
@TeleOp
public class FoundationCVConcept extends LinearOpMode
{
    OpenCvCamera camera;
    StageSwitchingPipeline stageSwitchingPipeline;
    static Scalar logoColor = new Scalar(255, 0, 0);

    private static final Point LOGO = new Point((640 /2) - 120 , 90);


    @Override
    public void runOpMode()
    {




        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCameraExample} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

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
        stageSwitchingPipeline = new StageSwitchingPipeline();
        camera.setPipeline(stageSwitchingPipeline);
        camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        telemetry.addData("Num contours found", 0);

        while (opModeIsActive())
        {
            telemetry.addData("Num contours found", stageSwitchingPipeline.getNumContoursFound());
            telemetry.update();
            sleep(100);
        }
    }

    /*
     * With this pipeline, we demonstrate how to change which stage of
     * is rendered to the viewport when the viewport is tapped. This is
     * particularly useful during pipeline development. We also show how
     * to get data from the pipeline to your OpMode.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat contoursOnFrameMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        int numContoursFound;
        Point thing;



        enum Stage
        {
            YCbCr_CHAN2,
            THRESHOLD,
            CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }

        private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }


        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();


            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 140, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            input.copyTo(contoursOnFrameMat);
            Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);


            Imgproc.putText(
                    input,
                    "CR Green CV",
                    LOGO,
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    3.0,
                    logoColor,
                    6

            );





            switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN2:
                {
                    return yCbCrChan2Mat;
                }

                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case CONTOURS_OVERLAYED_ON_FRAME:
                {
                    return contoursOnFrameMat;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

        public int getNumContoursFound()
        {
            return numContoursFound;
        }
    }
}


    /**
     * Saves a log to fileName specified and with format specified
     * <p/>
     * This function will not overwrite an existing log file, but append ".1", ".2", etc. if it already exists
     *
     * @param fileType Format to write file in
     */
    /*
public void saveAs(FileType fileType) {
    try {
        //Use correct filename for requested file type
        File f = Util.createFileOnDevice(fileDirectory, fileName + "." + fileType.toString(), false);
        String out = "";
        switch (fileType) {
            case JSON:
                Type logDataList = new TypeToken<List<LogData>>() {
                }.getType();
                Gson g = new Gson();
                out = g.toJson(logEntries, logDataList);
                break;
            case CSV:
                out = "time,tag,data\n";
                for (LogData l : logEntries) {
                    out += "\"" + l.time + "\"," + "\"" + l.tag + "\"," + "\"" + l.data + "\"\n";
                }
                break;
            case TEXT:
                for (LogData l : logEntries) {
                    out += l.time + ":[" + l.tag + "]" + l.data + "\n";
                }
                break;
        }
        FileWriter fw = new FileWriter(f.getAbsoluteFile());
        BufferedWriter bw = new BufferedWriter(fw);
        bw.write(out);
        bw.close();
    } catch (IOException e) {
        e.printStackTrace();
    }
}

 */