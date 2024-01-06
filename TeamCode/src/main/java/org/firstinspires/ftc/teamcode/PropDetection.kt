package org.firstinspires.ftc.teamcode

import org.opencv.core.*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCameraRotation

enum class PropPosition {
    Left, Center, Right
}

enum class PropColors {
    Red, Blue
}

class PropDetector(private val telemetry: Telemetry, private val colorToDetect: PropColors/*, private val frameType: FrameType = FrameType.Input*/) {
//    enum class FrameType {
//        Input,
//        ColorFiltered
//    }

    private val rects = listOf(
        Rect(Point(0.0, 450.0), Point(200.0, 720.0)),
        Rect(Point(300.0, 400.0), Point(900.0, 720.0)),
        Rect(Point(1000.0, 450.0), Point(1279.0, 720.0))
    )

    private val positionsMappedToRects = listOf(
        PropPosition.Left to rects[0],
        PropPosition.Center to rects[1],
        PropPosition.Right to rects[2]
    )

    private val red = Scalar(255.0, 0.0, 0.0) //100% Red
    private val green = Scalar(0.0, 255.0, 0.0) //100% Green
    private val blue = Scalar(0.0, 0.0, 255.0) //100% Blue
    private val white = Scalar(255.0, 255.0, 255.0)
    private val rectanglesMappedToBorderColors = listOf(
        positionsMappedToRects[0] to red,
        positionsMappedToRects[1] to green,
        positionsMappedToRects[2] to blue
    )

    private enum class Colors(val scalar: Scalar) {
        MinBlue(Scalar(100.0, 100.0, 100.0)),
        MaxBlue(Scalar(115.0, 255.0,255.0)),
        MinLowRed(Scalar(0.0,100.0, 100.0)),
        MaxLowRed(Scalar(25.0,255.0,255.0)),
        MinHighRed(Scalar(160.0, 100.0,100.0)),
        MaxHighRed(Scalar(255.0,255.0,255.0)),
    }

    @Volatile
    public var propPosition = PropPosition.Left

    private var mat = Mat()
    private var redLowMat = Mat()
    private var redHighMat = Mat()
    private var regions: List<Mat> = listOf()

    fun processFrame(frame: Mat): Mat {
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV)

        when (colorToDetect) {
            PropColors.Blue -> {
                Core.inRange(mat, Colors.MinBlue.scalar, Colors.MaxBlue.scalar, mat)
            }
            PropColors.Red -> {
                Core.inRange(mat, Colors.MinLowRed.scalar, Colors.MaxLowRed.scalar, redLowMat)
                Core.inRange(mat, Colors.MinHighRed.scalar, Colors.MaxHighRed.scalar, redHighMat)
                Core.bitwise_or(redLowMat, redHighMat, mat)
            }
        }
        regions = positionsMappedToRects.map { it ->
            mat.submat(it.second)
        }

        val values = regions.map { it ->
            Core.sumElems(it).`val`[0]
        }

        val leftValue = values[0]
        val centerValue = values[1]
        val rightValue = values[2]

        propPosition = if (leftValue >= rightValue && leftValue >= centerValue) {
            PropPosition.Left
        } else if (rightValue >= centerValue) {
            PropPosition.Right
        } else {
            PropPosition.Center
        }

        telemetry.addLine("propPosition: $propPosition")

//        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV)
//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB)

        rectanglesMappedToBorderColors.forEach { it ->
            val borderColor = if (it.first.first == propPosition) white else it.second
            Imgproc.rectangle(frame, it.first.second, borderColor, 3)
        }

        telemetry.update()
        return frame
    }
}


@Autonomous
class JamesVisionTest/** Change Depending on robot */: LinearOpMode() {


    /** Change Depending on robot */
    override fun runOpMode() {
        val opencv = OpenCvAbstraction(this)
        val tseDetector = PropDetector(telemetry, PropColors.Red)

        opencv.init(hardwareMap)
        opencv.internalCamera = false
        opencv.cameraName = "Webcam 1"
        opencv.cameraOrientation = OpenCvCameraRotation.UPRIGHT
        hardwareMap.allDeviceMappings.forEach { m ->
            println("HW: ${m.deviceTypeClass} ${m.entrySet().map{it.key}.joinToString(",")}")
        }
        opencv.onNewFrame(tseDetector::processFrame)

        tseDetector.propPosition
        waitForStart()
        /** AUTONOMOUS  PHASE */
    }

}