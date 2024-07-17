package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class SpikeProcessor implements VisionProcessor {
    public static Scalar blue = new Scalar(7,197,235,255);
    public static Scalar red = new Scalar(255,0,0,255);
    public static Scalar green = new Scalar(0,255,0,255);
    public static Scalar white = new Scalar(255,255,255,255);

    public Scalar lowerRed = new Scalar(0.0, 153.0, 48.0);
    public Scalar upperRed = new Scalar(255.0, 255.0, 141.0);
    public Scalar lowerBlue = new Scalar(10.1, 40.0, 136.0);
    public Scalar upperBlue = new Scalar(255.0, 129.0, 195.0);

    private final Mat yCrCb = new Mat();

    private final Size kSize = new Size(5, 5);
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kSize);
    private final Mat maskRed = new Mat(), maskBlue = new Mat();
    public Rect outer = new Rect(10, 275, 100, 100), center  = new Rect(300, 200, 100, 100);

    private Rect redRect = new Rect(), blueRect = new Rect();
    private double redMin = 140, blueMin = 160;

    private double min = 100,max = 1000000;
    private double horizon = 160;
    public static Mode mode = Mode.BLUE;

    private Mat submat_outer = new Mat(), submat_center;

    public enum Mode {
        RED,
        BLUE,
        BOTH
    }

    public Location location = Location.NONE;

    public enum Location {
        OUTER,
        CENTER,
        NONE
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.dilate(frame, frame, kernel);
        Imgproc.erode(frame, frame, kernel);

        submat_outer = frame.submat(outer);
        submat_center = frame.submat(center);

        if(mode.equals(Mode.BLUE) || mode.equals(Mode.BOTH)) {
            Scalar mean1 = Core.mean(submat_outer);
//            telemetry.addData("Outer avg", mean1);
            Scalar mean2 = Core.mean(submat_center);
//            telemetry.addData("Center avg", mean2);

            if(mean1.val[2] > mean2.val[2]) {
//                telemetry.addData("Position", "Outer");
                location = Location.OUTER;
                Imgproc.circle(frame, new Point(outer.x + outer.width / 2.0, outer.y + outer.height / 2.0), 3, red);
            } else if (mean2.val[2] > blueMin) {
//                telemetry.addData("Position", "Center");
                Imgproc.circle(frame, new Point(center.x + center.width / 2.0, center.y + center.height / 2.0), 3, red);
                location = Location.CENTER;
            } else {
//                telemetry.addData("Position", "Not Seen");
                location = Location.NONE;
            }
        }

        if(mode.equals(Mode.RED) || mode.equals(Mode.BOTH)) {
            Scalar mean1 = Core.mean(submat_outer);
//            telemetry.addData("Outer avg", mean1);
            Scalar mean2 = Core.mean(submat_center);
//            telemetry.addData("Center avg", mean2);

            if(mean1.val[0] > mean2.val[0])
//                telemetry.addData("Position", "Outer");
                location = Location.OUTER;
            else if (mean2.val[0] > redMin) {
//                telemetry.addData("Position", "Center");
                location = Location.CENTER;
            } else {
//                telemetry.addData("Position", "Not Seen");
                location = Location.NONE;
            }
        }

        Imgproc.rectangle(frame, outer, new Scalar(0, 255, 0));
        Imgproc.rectangle(frame, center, new Scalar(0, 255, 0));
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
