package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
public class Right extends LinearOpMode {
    Materials M = new Materials();
    BackgroundThread BT = new BackgroundThread();
    PerformingThread PT = new PerformingThread();

    public static double TranslationalKp = 0.14, TranslationalKd = 0.7,
            TurnKp = 0.026, TurnKd = 0.08, AccelKp = 1.6, WallPushingPower = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        BT.start();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready!");
            telemetry.update();
        }
        telemetry.addLine();
        telemetry.update();
        PT.start();

        while (!isStopRequested()) ;

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class PerformingThread extends Thread {
        DrivingThread DT = new DrivingThread();

        public void run() {
            DT.start();

            while (!isStopRequested()) ;
        }

        class DrivingThread extends Thread {
            public void run() {
                Vector2D LastXYErrorVec = new Vector2D();
                double LastAngleError = 0;
                while (!isStopRequested()) {
                    Vector2D XYErrorVec = new Vector2D(M.TargetX - M.Drive.getPoseEstimate().getX(), M.TargetY - M.Drive.getPoseEstimate().getY());
                    boolean LocalWallPushing = M.WallPushing;
                    Vector2D SFPowerVec = (LocalWallPushing ?
                            new Vector2D(XYErrorVec.x * TranslationalKp + (XYErrorVec.x - LastXYErrorVec.x) * TranslationalKd, -WallPushingPower) :
                            XYErrorVec.getMultiplied(TranslationalKp).getAdded(XYErrorVec.getSubtracted(LastXYErrorVec).getMultiplied(TranslationalKd)))
                            .getRotatedBy(-M.Drive.getPoseEstimate().getHeading());
                    if (SFPowerVec.getLength() > 1) SFPowerVec.normalize();
                    LastXYErrorVec.set(XYErrorVec);

                    double AngleError = M.MinAngleError(M.TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()));
                    M.Drive.setWeightedDrivePower(new Pose2d(SFPowerVec.x, SFPowerVec.y,
                            M.Limit(AngleError * TurnKp + (AngleError - LastAngleError) * TurnKd, 1)
                    ).times(LocalWallPushing ? 1 : Math.min(1, M.AccelTimer.seconds() * AccelKp)));
                    M.Drive.update();
                    LastAngleError = AngleError;
                }
                M.Drive.setMotorPowers(0, 0, 0, 0);
            }
        }
    }

    class BackgroundThread extends Thread {
        public void run() {
            M.Swing.setPosition(Materials.SwingInsidePos);
            M.SetPaw(Materials.PawFoldPos, 0);
            M.LowerClaw.setPosition(Materials.LowerClawMidPos);

            while (!isStopRequested()) {
                M.Extender.setPower(M.NeedToResetExtender ? Materials.ExtenderResetPower : M.ExtenderToPosPower());
                if (M.NeedToResetExtender && M.ExtenderDownEnd.isPressed()) {
                    M.ExtenderDownPos = M.Extender.getCurrentPosition();
                    M.TargetExtenderPos = 0;
                    M.NeedToResetExtender = false;
                }

                M.LiftUpdate();
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
            M.SetLiftPower(0);
        }
    }
}
