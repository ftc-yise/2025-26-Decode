package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.opencv.core.Rect;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class webcamTest {
    @TeleOp(name = "webcamTest")
    public static class WebcamTest extends LinearOpMode {

        private VisionProcessor processor;
        private webcamTest webcam;

        @Override
        public void runOpMode() {
            waitForStart();

            VisionPortal visionPortal = new VisionPortal.Builder()
                    .setCamera((CameraName) webcam)
                    .addProcessor(processor)
                    .enableLiveView(true)
                    .build();
            abstract class YellowProcessor implements VisionProcessor {

                private Rect largestRect = null;

                @Override
                public void init(int width, int height, CameraCalibration calibration) {
                }

                @Override
                public Object processFrame(Mat frame, long captureTimeNanos) {

                    Mat hsv = new Mat();
                    Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

                    Scalar lower = new Scalar(20,100,100);
                    Scalar upper = new Scalar(35,255,255);

                    Mat mask = new Mat();
                    Core.inRange(hsv, lower, upper, mask);

                    List<MatOfPoint> contours = new ArrayList<>();

                    Imgproc.findContours(
                            mask,
                            contours,
                            new Mat(),
                            Imgproc.RETR_EXTERNAL,
                            Imgproc.CHAIN_APPROX_SIMPLE);

                    double largestArea = 0;
                    largestRect = null;

                    for (MatOfPoint contour : contours) {

                        org.opencv.core.Rect rect = Imgproc.boundingRect(contour);

                        if (rect.area() > largestArea) {
                            largestArea = rect.area();
                            largestRect = rect;
                        }
                    }

                    if (largestRect != null) {
                        Imgproc.rectangle(frame,largestRect,new Scalar(0,255,0),3);
                    }

                    return null;

                }

                @Override
                public void onDrawFrame(
                        Canvas canvas,
                        int onscreenWidth,
                        int onscreenHeight,
                        float scaleBmpPxToCanvasPx,
                        float scaleCanvasDensity,
                        Object userContext) {
                }
            }
        }
    }
}