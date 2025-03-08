package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous
public class CameraIntake extends LinearOpMode {
    ElapsedTime AccelTimer = new ElapsedTime(), CalmDownTimer = new ElapsedTime();
    double TargetX = 0, TargetY = 0, TargetAngle = 90;
    boolean WallPushing = false, Red = true;

    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        boolean APressed = false;
        while ((M.NeedToResetExtender || !isStarted()) && !isStopRequested()) {
            if (gamepad1.a && !APressed) {
                APressed = true;
                Red = !Red;
            }
            if (!gamepad1.a) APressed = false;
        }
        new DrivingThread().start();
        M.InitOpenCV(hardwareMap, Red);

        while (!gamepad1.dpad_right && !isStopRequested()) ;

        Pose2d LocalSamplePose = M.SDP.SamplePose;
        M.Webcam.closeCameraDevice();
        if (LocalSamplePose != null) {
            AccelTimer.reset();
            TargetX = M.Drive.getPoseEstimate().getX() + Right.SampleXC + LocalSamplePose.getX() * Right.SampleXKp;
            M.TargetExtenderPos = Math.min(Right.SampleYC + LocalSamplePose.getY() * Right.SampleYKp, Materials.ExtenderMaxPos);
            M.Swing.setPosition(Materials.SwingPreparePos);
            M.Wait(200);
            double PawTargetAngle = Math.toDegrees(LocalSamplePose.getHeading());
            M.SetPaw(Materials.PawFoldPos, (PawTargetAngle > 90 ? PawTargetAngle - 90 : -(90 - PawTargetAngle)));
            M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
            CalmDownTimer.reset();
            while (CalmDownTimer.milliseconds() < 100 && !isStopRequested()) {
                if (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                        TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 ||
                        Math.abs(M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()))) > 5 ||
                        Math.abs(M.ExtenderPosError()) > 20) CalmDownTimer.reset();
                CameraIntake.this.sleep(10);
            }
            M.Swing.setPosition(Materials.SwingBottomPos);
            M.Wait(70);
            M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
            M.Wait(150);
            M.Swing.setPosition(Materials.SwingInsidePos);
            M.Wait(150);
            M.TargetExtenderPos = 400;
            M.SetPaw(Materials.PawThrowPos, 0);
            M.Wait(200);
        }
    }

    class DrivingThread extends Thread {
        public void run() {
            Vector2D LastXYErrorVec = new Vector2D();
            double LastAngleError = 0;
            while (!isStopRequested()) {
                Vector2D XYErrorVec = new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(), TargetY - M.Drive.getPoseEstimate().getY());
                boolean LocalWallPushing = WallPushing;
                Vector2D SFPowerVec = (LocalWallPushing ? new Vector2D(XYErrorVec.x * Right.TranslationalKp +
                        (XYErrorVec.x - LastXYErrorVec.x) * Right.TranslationalKd, -0.4) :
                        XYErrorVec.getMultiplied(Right.TranslationalKp).getAdded(XYErrorVec.getSubtracted(LastXYErrorVec).getMultiplied(Right.TranslationalKd)))
                        .getRotatedBy(-M.Drive.getPoseEstimate().getHeading());
                if (SFPowerVec.getLength() > 1) SFPowerVec.normalize();
                LastXYErrorVec.set(XYErrorVec);

                double AngleError = M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()));
                M.Drive.setWeightedDrivePower(new Pose2d(SFPowerVec.x, SFPowerVec.y,
                        M.Limit(AngleError * Right.TurnKp + (AngleError - LastAngleError) * Right.TurnKd, 1)
                ).times(LocalWallPushing ? 1 : Math.min(1, AccelTimer.seconds() * 2)));
                M.Drive.update();
                LastAngleError = AngleError;
            }
            M.Drive.setMotorPowers(0, 0, 0, 0);
        }
    }

    class BackgroundThread extends Thread {
        public void run() {
            M.Swing.setPosition(Materials.SwingInsidePos);
            M.SetPaw(Materials.PawFoldPos, 0);
            M.LowerClaw.setPosition(Materials.LowerClawMidPos);

            M.ExtenderResetTimer.reset();
            while (!isStopRequested()) {
                M.Extender.setPower(M.NeedToResetExtender ? Materials.ExtenderResetPower : M.ExtenderToPosPower());
                if (M.NeedToResetExtender && M.ExtenderDownEnd.isPressed()) {
                    if (M.ExtenderResetTimer.milliseconds() > Materials.ExtenderResetMilliseconds) {
                        M.ExtenderDownPos = M.Extender.getCurrentPosition();
                        M.TargetExtenderPos = 0;
                        M.ExtenderResetTimer.reset();
                        M.NeedToResetExtender = false;
                    }
                } else M.ExtenderResetTimer.reset();

                telemetry.addLine(Red ? "Red" : "Blue");
                telemetry.addData("NeedToResetExtender", M.NeedToResetExtender);
                telemetry.update();
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
        }
    }
}
