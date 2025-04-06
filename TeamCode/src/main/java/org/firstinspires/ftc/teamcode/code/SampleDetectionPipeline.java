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

    public static Scalar LowerRed1 = new Scalar(0, 75, 46), UpperRed1 = new Scalar(10, 255, 255),
            LowerRed2 = new Scalar(160, 0, 0), UpperRed2 = new Scalar(180, 255, 255),
            LowerBlue = new Scalar(100, 60, 0), UpperBlue = new Scalar(140, 255, 255);
    public static double TrapTopX = 50, TrapTopY = 40, TrapBottomX = 100, TrapBottomY = 200, MinArea = 2000, MaxArea = 6000;
    public static Point[] SourcePoints = {new Point(0, 0), new Point(320, 0),
            new Point(440, 240), new Point(-120, 240)},
            DestPoints = {new Point(0, 0), new Point(320, 0),
                    new Point(320, 240), new Point(0, 240)};

    public Pose2d SamplePose = null;
    public double SampleArea = 0;

    @Override
    public Mat processFrame(Mat Input) {
        MatOfPoint2f SourceMat = new MatOfPoint2f(SourcePoints);
        MatOfPoint2f DestMat = new MatOfPoint2f(DestPoints);
        Mat PerspectiveTransformMatrix = Imgproc.getPerspectiveTransform(SourceMat, DestMat);
        SourceMat.release();
        DestMat.release();
        Imgproc.warpPerspective(Input, Input, PerspectiveTransformMatrix, Input.size());
        PerspectiveTransformMatrix.release();

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

        Point[] TrapPoints = {new Point(TrapTopX, TrapTopY), new Point(320 - TrapTopX, TrapTopY),
                new Point(320 - TrapBottomX, TrapBottomY), new Point(TrapBottomX, TrapBottomY)};
        for (int i = 0; i < 4; i++) {
            Imgproc.line(Input, TrapPoints[i], TrapPoints[(i + 1) % 4], new Scalar(255, 255, 255), 1);
        }
        Imgproc.line(Input, new Point(160, 0), new Point(160, 240), new Scalar(255, 255, 255), 1);
        Imgproc.line(Input, new Point(0, 120), new Point(320, 120), new Scalar(255, 255, 255), 1);

        RotatedRect TargetRect = null;
        for (MatOfPoint Contour : Contours) {
            MatOfPoint2f Contour2f = new MatOfPoint2f(Contour.toArray());
            Contour.release();
            RotatedRect DetectedRect = Imgproc.minAreaRect(Contour2f);
            Contour2f.release();

            if (DetectedRect.size.area() >= MinArea && DetectedRect.size.area() <= MaxArea && InsideTrapezoid(DetectedRect.center, TrapPoints)) {
                Point[] DetectedRectPoints = new Point[4];
                DetectedRect.points(DetectedRectPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(Input, DetectedRectPoints[i], DetectedRectPoints[(i + 1) % 4], new Scalar(66, 228, 245), 2);
                }

                if (TargetRect != null) {
                    if (DetectedRect.center.x < TargetRect.center.x)
                        TargetRect = DetectedRect;
                } else TargetRect = DetectedRect;
            }
        }

        if (TargetRect != null) {
            SamplePose = new Pose2d(TargetRect.center.x - 160, -TargetRect.center.y + 120,
                    Math.toRadians(-TargetRect.angle + (TargetRect.size.width > TargetRect.size.height ? 180 : 90)));
            SampleArea = TargetRect.size.area();

            Imgproc.arrowedLine(Input, new Point(TargetRect.center.x, TargetRect.center.y),
                    new Point(TargetRect.center.x + (25 * Math.cos(SamplePose.getHeading())),
                            TargetRect.center.y + (25 * -Math.sin(SamplePose.getHeading()))),
                    new Scalar(66, 255, 66), 2);
        } else SamplePose = null;

        return Input;
    }

    private boolean InsideTrapezoid(Point DetectedRectCenter, Point[] TrapPoints) {
        return Left(TrapPoints[0], TrapPoints[1], DetectedRectCenter) &&
                Left(TrapPoints[1], TrapPoints[2], DetectedRectCenter) &&
                Left(TrapPoints[2], TrapPoints[3], DetectedRectCenter) &&
                Left(TrapPoints[3], TrapPoints[0], DetectedRectCenter);
    }

    private boolean Left(Point A, Point B, Point C) {
        return ((B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x)) > 0;
    }
}
