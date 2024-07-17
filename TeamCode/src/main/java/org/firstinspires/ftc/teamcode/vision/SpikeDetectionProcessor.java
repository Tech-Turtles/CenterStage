package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.inRange;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class SpikeDetectionProcessor implements VisionProcessor {

    public static Scalar blue = new Scalar(7,197,235,255);
    public static Scalar red = new Scalar(255,0,0,255);
    public static Scalar green = new Scalar(0,255,0,255);
    public static Scalar white = new Scalar(255,255,255,255);

    public Scalar lowerRed = new Scalar(0.0, 157.3, 0.0);
    public Scalar upperRed = new Scalar(157.3, 255.0, 255.0);
    public Scalar lowerBlue = new Scalar(0.0, 43.1, 0.0);
    public Scalar upperBlue = new Scalar(148.8, 123.3, 255.0);

    private final Mat yCrCb = new Mat();

    private final Size kSize = new Size(5, 5);
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kSize);
    private final Mat maskRed = new Mat(), maskBlue = new Mat();
    private final List<MatOfPoint> redContours    = new ArrayList<>();
    private final List<MatOfPoint> blueContours   = new ArrayList<>();

    private Rect redRect = new Rect(), blueRect = new Rect();

    private double min = 100,max = 1000000;
    private double horizon = 160;
    public Mode mode = Mode.BOTH;
    public Location location = Location.NONE;

    public enum Mode {
        RED,
        BLUE,
        BOTH
    }

    public enum Location {
        CENTER,
        OUTER,
        NONE
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.dilate(yCrCb, yCrCb, kernel);
        Imgproc.erode(yCrCb, yCrCb, kernel);

        if(mode.equals(Mode.BLUE) || mode.equals(Mode.BOTH)) {
            inRange(yCrCb, lowerBlue, upperBlue, maskBlue);
            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            blueContours.removeIf(c -> Imgproc.contourArea(c) > max || Imgproc.contourArea(c) < min || Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
            Imgproc.drawContours(frame, blueContours, -1, blue, 2);

            blueContours.sort(Collections.reverseOrder(Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).area())));
            try {
                MatOfPoint biggestRedContour = blueContours.get(0);
                blueRect = Imgproc.boundingRect(biggestRedContour);
                Imgproc.putText(frame, "Blue Rect", new Point(blueRect.x, blueRect.y < 10 ? (blueRect.y+blueRect.height+20) : (blueRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, green, 1);
                Imgproc.circle(frame, new Point(blueRect.x + (blueRect.width/2.0), blueRect.y + (blueRect.height/2.0)), 3, green, 4);
                if(blueRect.area() > 4000) {
                    if(blueRect.x > 175)
                        location = Location.OUTER;
                    else
                        location = Location.CENTER;
                } else {
                    location = Location.NONE;
                }
            } catch (IndexOutOfBoundsException ignore) {}
            maskBlue.release();
            blueContours.clear();
        }

        if(mode.equals(Mode.RED) || mode.equals(Mode.BOTH)) {
            inRange(yCrCb, lowerRed, upperRed, maskRed);
            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            redContours.removeIf(c -> Imgproc.contourArea(c) > max || Imgproc.contourArea(c) < min || Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
            Imgproc.drawContours(frame, redContours, -1, red, 2);

            redContours.sort(Collections.reverseOrder(Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).area())));
            try {
                MatOfPoint biggestRedContour = redContours.get(0);
                redRect = Imgproc.boundingRect(biggestRedContour);
                Imgproc.putText(frame, "Red Rect", new Point(redRect.x, redRect.y < 10 ? (redRect.y + redRect.height + 20) : (redRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, green, 1);
                Imgproc.circle(frame, new Point(redRect.x + (redRect.width / 2.0), redRect.y + (redRect.height / 2.0)), 3, green, 4);
                if(redRect.area() > 4000) {
                    if(redRect.x > 175)
                        location = Location.OUTER;
                    else
                        location = Location.CENTER;
                } else {
                    location = Location.NONE;
                }
            } catch (IndexOutOfBoundsException ignore) {}
            maskRed.release();
            redContours.clear();
        }

        Imgproc.line(frame, new Point(0, horizon), new Point(640, horizon), green, 2);

        yCrCb.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Rect getRedRect() {
        return redRect;
    }

    public Rect getBlueRect() {
        return blueRect;
    }
}
