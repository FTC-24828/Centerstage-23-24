package org.firstinspires.ftc.teamcode.common.vision;

import static org.firstinspires.ftc.teamcode.common.hardware.Global.DEBUG;
import static org.firstinspires.ftc.teamcode.common.hardware.Global.SIDE;
import static org.firstinspires.ftc.teamcode.common.hardware.Global.Side;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import static org.firstinspires.ftc.teamcode.common.hardware.Global.*;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.util.WMath;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class PropPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> frame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    public Global.PropLocation location;

    public List<Integer> blue_range =  Arrays.asList(0, 12, 127, 17, 255, 255);
    public List<Integer> red_range = Arrays.asList(106, 52, 0, 128, 255, 255);
    public List<Integer> filter_range;

    public double red_threshold = 0.2;
    public double blue_threshold = 2.5;
    public double threshold = 0;

    double left_white;
    double center_white;
    double right_white;

    Mat mask = new Mat();
    Mat final_mat = new Mat();
    Mat submat = new Mat();

    ArrayList<Rect> bounding_box = new ArrayList<>();

    //for displaying with ftc dashboard
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(frame.get()));
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frame.set(Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888));
        if (SIDE == Side.BLUE) {
            threshold = blue_threshold;
            filter_range = blue_range;
        } else if (SIDE == Side.RED) {
            threshold = red_threshold;
            filter_range = red_range;
        } else {
            throw new EnumConstantNotPresentException(Global.Side.class, "Global.SIDE not initialized");
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        mask = frame.clone();
        final_mat = frame.clone();

        Imgproc.GaussianBlur(mask, mask, new Size(11, 11), 0.0);
        mask = filterColor(mask);

        //clean up image
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        //get bounding box of contours
        processContours(mask);

        //sort left to right by x value
        bounding_box.sort(Comparator.comparingInt(a -> a.x));
        //get the
        left_white = getWhiteSum(mask, bounding_box.get(0)) / 1000000;
        center_white = getWhiteSum(mask, bounding_box.get(1)) / 1000000;
        right_white = getWhiteSum(mask, bounding_box.get(2)) / 1000000;

        setPropLocation(left_white, center_white, right_white);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (DEBUG) {

        }
    }

    public Mat filterColor(Mat src) {
        Mat HSV = new Mat();
        Imgproc.cvtColor(src, HSV, Imgproc.COLOR_RGB2HSV);
        Scalar lower = new Scalar(filter_range.get(0), filter_range.get(1), filter_range.get(2));
        Scalar upper = new Scalar(filter_range.get(3), filter_range.get(4), filter_range.get(5));
        Core.inRange(HSV, lower, upper, HSV);
        return HSV;
    }

    private void processContours(Mat src) {
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(src, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        bounding_box.clear();

        for (int i = 0; i < contours.size(); i++) {
            if (Imgproc.contourArea(contours.get(i)) > 5000) {
                bounding_box.add(Imgproc.boundingRect(contours.get(i)));
            }
        }
    }

    private double getWhiteSum(Mat src, Rect box) {
        submat = src.submat(box);
        Scalar color =  Core.sumElems(submat);
        return color.val[0];
    }

    private void setPropLocation(double left, double center, double right) {
//        if (left > threshold) {
//            location = PropLocation.LEFT;
//        } else if (center > threshold) {
//            location = PropLocation.CENTER;
//        } else {
//            location = PropLocation.RIGHT
//        }
        double highest = WMath.max(left, center, right);
        if (highest - left < 0.1) {
            location = PropLocation.LEFT;
        } else if (highest - center < 0.1) {
            location = PropLocation.CENTER;
        } else {
            location = PropLocation.RIGHT;
        }
    }

    public PropLocation getPropLocation () { return location; }

    public int getNumberOfDetection() {
        return bounding_box.size();
    }

}
