package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class SampleDetectionPipeline extends OpenCvPipeline {
    private boolean Red;

    public SampleDetectionPipeline(boolean Red) {
        this.Red = Red;
    }

    public static Scalar LowerRed1 = new Scalar(0, 100, 100), UpperRed1 = new Scalar(10, 255, 255),
            LowerRed2 = new Scalar(160, 100, 100), UpperRed2 = new Scalar(180, 255, 255),
            LowerBlue = new Scalar(100, 75, 75), UpperBlue = new Scalar(140, 255, 255);
    public static double LeftBorder = 40, RightBorder = 40, UpBorder = 40, BottomBorder = 40,
            MinArea = 4000, MaxArea = 13000;

    public Pose2d SamplePose = null;

    @Override
    public Mat processFrame(Mat Input) {
        Mat HSV = new Mat();
        Imgproc.cvtColor(Input, HSV, Imgproc.COLOR_RGB2HSV);

        Mat Color = new Mat();
        if (Red) {
            Mat Red1 = new Mat();
            Core.inRange(HSV, LowerRed1, UpperRed1, Red1);
            Mat Red2 = new Mat();
            Core.inRange(HSV, LowerRed2, UpperRed2, Red2);
            HSV.release();

            Core.bitwise_or(Red1, Red2, Color);
            Red1.release();
            Red2.release();
        } else {
            Core.inRange(HSV, LowerBlue, UpperBlue, Color);
            HSV.release();
        }

        List<MatOfPoint> Contours = new ArrayList<>();
        Mat Hierarchy = new Mat();
        Imgproc.findContours(Color, Contours, Hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Color.release();
        Hierarchy.release();

        RotatedRect LargestRect = null;
        for (MatOfPoint Contour : Contours) {
            MatOfPoint2f Contour2f = new MatOfPoint2f(Contour.toArray());
            Contour.release();
            RotatedRect DetectedRect = Imgproc.minAreaRect(Contour2f);
            Contour2f.release();

            if (DetectedRect.size.area() > MinArea && DetectedRect.size.area() < MaxArea &&
                    DetectedRect.center.x > LeftBorder && DetectedRect.center.x < 320 - RightBorder &&
                    DetectedRect.center.y > UpBorder && DetectedRect.center.y < 240 - BottomBorder) {
                if (LargestRect != null) {
                    if (DetectedRect.size.area() > LargestRect.size.area())
                        LargestRect = DetectedRect;
                } else LargestRect = DetectedRect;
            }
        }

        Imgproc.rectangle(Input, new Point(LeftBorder, UpBorder),
                new Point(320 - RightBorder, 240 - BottomBorder), new Scalar(255, 0, 0), 1);
        if (LargestRect != null) {
            SamplePose = new Pose2d(LargestRect.center.x - LeftBorder, 240 - LargestRect.center.y - BottomBorder,
                    Math.toRadians(-LargestRect.angle + (LargestRect.size.width > LargestRect.size.height ? 180 : 90)));

            Imgproc.arrowedLine(Input, new Point(LargestRect.center.x, LargestRect.center.y),
                    new Point(LargestRect.center.x + (30 * Math.cos(SamplePose.getHeading())),
                            LargestRect.center.y + (30 * -Math.sin(SamplePose.getHeading()))),
                    new Scalar(0, 255, 0), 2);
        } else SamplePose = null;

        return Input;
    }
}
