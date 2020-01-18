package org.firstinspires.ftc.teamcode.CRVuforia.EasyOpenCv


import org.opencv.core.*
import org.opencv.imgproc.Imgproc

class NaiveRectangleSamplingSkystoneDetectionPipeline : Init3BlockDetection() {
    private val matYCrCb = Mat()
    private val matCb = Mat()

    private val samplingRectWidth = 17
    private val samplingRectHeight = 17
    private val samplingRectColor = Scalar(0.0, 255.0, 0.0)
    private val samplingRectThickness = 1

    private val samplingCircleRadius = 5
    private val samplingCircleColor = Scalar(225.0, 52.0, 235.0)
    private val samplingCircleThickness = -1

    private val sampleSectionDivLength = 60
    private val sampleSectionDivColor = Scalar(00.0, 255.0, 40.0)
    private val sampleSectionDivThickness = 4
    private val sectionDivHalfLenght = sampleSectionDivLength / 2

    private val logoColor = Scalar(0.0, 255.0, 190.0)
    private val LOGO = Point((640 / 2 - 150).toDouble(), 90.0)

    private val samplePointPercentages = arrayOf(
            arrayOf(.18, .59), arrayOf(.5, .59), arrayOf(.82, .59)
    )

    private val samplePoints = samplePointPercentages.map {
        arrayOf(
                Point(
                        it[0] * width - samplingRectWidth / 2,
                        it[1] * height - samplingRectHeight / 2
                ),
                Point(
                        it[0] * width + samplingRectWidth / 2,
                        it[1] * height + samplingRectHeight / 2
                )
        )
    }

    override fun processFrame(input: Mat): Mat {

        // Convert the image from RGB to
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb)

        // Extract the Cb channel from the image
        Core.extractChannel(matYCrCb, matCb, 2)

        // Then the sample areas from the Cb channel
        var subMats = samplePoints.map {
            matCb.submat(Rect(it[0], it[1]))
        }


        // Average the sample areas
        val avgSamples = subMats.map {
            Core.mean(it).`val`[0]
        }


        // Draw rectangles around the sample areas
        samplePoints.forEach {
            Imgproc.rectangle(input, it[0], it[1], samplingRectColor, samplingRectThickness)
        }

        // Figure out which sample zone had the lowest contrast from blue (lightest color)
        val max = avgSamples.max()

        // Draw a circle on the detected skystone
        detectedSkystonePosition = avgSamples.indexOf(max)

        val detectedPoint = samplePoints[detectedSkystonePosition]
        Imgproc.circle(
                input,
                Point(
                        (detectedPoint[0].x + detectedPoint[1].x) / 2,
                        (detectedPoint[0].y + detectedPoint[1].y) / 2
                ),
                samplingCircleRadius,
                samplingCircleColor,
                samplingCircleThickness
        )

        //Add Text over the detected position
        Imgproc.putText(
                input,
                "FOUND!",
                Point(
                        ((detectedPoint[0].x + detectedPoint[1].x) / 2) - 50,
                        ((detectedPoint[0].y + detectedPoint[1].y) / 2) - 40
                ),
                Imgproc.FONT_HERSHEY_PLAIN,
                1.5,
                Scalar(0.00, 255.0, 0.0),
                3

        )

        //Add section dividers for clarity

        Imgproc.line(
                input,
                Point(
                        width * 0.333333,
                        height - 20.0
                ),
                Point(
                        width * 0.33333333,
                        20.0
                ),
                sampleSectionDivColor,
                sampleSectionDivThickness
        )


        Imgproc.line(
                input,
                Point(
                        width * 0.6666666,
                        height - 20.0
                ),
                Point(
                        width * 0.66666666,
                        20.0
                ),
                sampleSectionDivColor,
                sampleSectionDivThickness
        )

        //Add text for marketing
        Imgproc.putText(
                input,
                "CR Green CV",
                LOGO,
                Imgproc.FONT_HERSHEY_PLAIN,
                1.75,
                logoColor,
                5

        )





        // Free the allocated submat memory
        subMats.forEach {
            it.release()
        }

        return input
    }
}