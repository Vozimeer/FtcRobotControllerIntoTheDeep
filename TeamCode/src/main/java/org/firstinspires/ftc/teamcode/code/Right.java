package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Right extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready!");
            telemetry.update();
        }
        telemetry.addLine();
        telemetry.update();
        new PerformingThread().start();

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class PerformingThread extends Thread {
        public void run() {
            new DrivingThread().start();

            while (!isStopRequested()) ;
        }

        class DrivingThread extends Thread {
            public void run() {
                Vector2D LastXYErrorVec = new Vector2D();
                double LastAngleError = 0;
                while (!M.StopRequested) {
                    Vector2D XYErrorVec = new Vector2D(M.TargetX - M.Drive.getPoseEstimate().getX(), M.TargetY - M.Drive.getPoseEstimate().getY());
                    boolean LocalWallPushing = M.WallPushing;
                    Vector2D SFPowerVec = (LocalWallPushing ? new Vector2D(XYErrorVec.x * Materials.TranslationalKp +
                            (XYErrorVec.x - LastXYErrorVec.x) * Materials.TranslationalKd, -Materials.WallPushingPower) :
                            XYErrorVec.getMultiplied(Materials.TranslationalKp)
                                    .getAdded(XYErrorVec.getSubtracted(LastXYErrorVec).getMultiplied(Materials.TranslationalKd)))
                            .getRotatedBy(-M.Drive.getPoseEstimate().getHeading());
                    if (SFPowerVec.getLength() > 1) SFPowerVec.normalize();
                    LastXYErrorVec.set(XYErrorVec);

                    double AngleError = M.MinAngleError(M.TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()));
                    M.Drive.setWeightedDrivePower(new Pose2d(SFPowerVec.x, SFPowerVec.y,
                            M.Limit(AngleError * Materials.TurnKp + (AngleError - LastAngleError) * Materials.TurnKd, 1)
                    ).times(LocalWallPushing ? 1 : Math.min(1, M.AccelTimer.seconds() * Materials.AccelKp)));
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
