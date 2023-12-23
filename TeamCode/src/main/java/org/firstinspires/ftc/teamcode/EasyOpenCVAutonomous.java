package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous
public class EasyOpenCVAutonomous extends OpMode {
    int xCameraLength = 1280;
    int yCameraLength = 720;
    OpenCvWebcam webcam1 = null;
    @Override
    public void init(){
        WebcamName myWebcam = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(myWebcam, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(xCameraLength, yCameraLength, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public void loop(){

    }
    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            //telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1,1,xCameraLength/3 - 1, yCameraLength-1);
            Rect midRect = new Rect(xCameraLength/3,1,xCameraLength/3 - 1, yCameraLength-1);
            Rect rightRect = new Rect(xCameraLength*2/3,1, xCameraLength/3 - 1, yCameraLength-1);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,rectColor,2);
            Imgproc.rectangle(outPut,midRect,rectColor,2);
            Imgproc.rectangle(outPut,rightRect,rectColor,2);

            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2); //1 is red and 2 is blue
            Core.extractChannel(midCrop, midCrop, 2);
            Core.extractChannel(rightCrop,rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            telemetry.addData("right color", rightavgfin);
            telemetry.addData("mid color", midavgfin);
            telemetry.addData("left color", leftavgfin);
            if(leftavgfin > rightavgfin && leftavgfin > midavgfin){
                telemetry.addLine("left");
            }else if (rightavgfin > leftavgfin && rightavgfin > midavgfin){
                telemetry.addLine("right");
            }else if(midavgfin >= leftavgfin && midavgfin >= rightavgfin){
                telemetry.addLine("middle");
            }
            return (outPut);
        }
    }
}
