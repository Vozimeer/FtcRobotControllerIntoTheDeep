package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class RightFastAndFuriousCamera extends LinearOpMode {
    public static double TranslationalKp = 0.12, TranslationalKd = 0.6, TurnKp = 0.02, TurnKd = 0.06,
            AccelKp = 1.6, WallPushingPower = 0.4,

    FirstClipY = 34, FirstSampleX = 58, FirstTwoSamplesY = 10, ExtenderFirstTwoSamplesPos = 520,
            SamplesFastThrowExtenderPos = 200, SecondSampleX = 67,
            ThirdSampleY = 16, ThirdSampleAngle = 65, ExtenderThirdSamplePos = 450, PawThirdSampleAngle = 34,
            ClipXStep = 2, ClipPrepareY = 25, ClipY = 36, WallIntakeX = 54, WallIntakePrepareY = 2,
            SampleXMultiply = 0.04, SampleYC = 200, SampleYMultiply = 0.081, SampleYPow = 2.05;

    public static int FirstCLipMilliseconds = 1200;

    ElapsedTime AccelTimer = new ElapsedTime();
    double TargetX = 0, TargetY = 0, TargetAngle = 90;
    boolean WallPushing = false, Red = true;

    Materials M = new Materials();
    LowerBackgroundThread LBT = new LowerBackgroundThread();
    UpperBackgroundThread UBT = new UpperBackgroundThread();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        boolean APressed = false;
        while ((M.NeedToResetExtender || M.NeedToResetLift || !isStarted()) && !isStopRequested()) {
            if (gamepad1.a && !APressed) {
                APressed = true;
                Red = !Red;
            }
            if (!gamepad1.a) APressed = false;
        }
        new DrivingThread().start();
        LBT.SetAction("InitOpenCV");
        LBT.start();
        M.Wait(4000);

        Pose2d LocalSamplePose = M.SDP.SamplePose;
        if (LocalSamplePose != null && !LBT.isAlive()) {
            AccelTimer.reset();
            TargetX += LocalSamplePose.getX() * SampleXMultiply;
            M.TargetExtenderPos = Math.min(SampleYC + Math.pow(LocalSamplePose.getY() * SampleYMultiply, SampleYPow), 550);
            M.Swing.setPosition(Materials.SwingPreparePos);
            double PawTargetAngle = Math.toDegrees(LocalSamplePose.getHeading());
            M.SetPaw(Materials.PawFoldPos, (PawTargetAngle > 90 ? PawTargetAngle - 90 : -(90 - PawTargetAngle)));
            M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
            M.Wait(2000);
            while (TargetX - M.Drive.getPoseEstimate().getX() > 2 &&
                    M.ExtenderPosError() > 20 && !isStopRequested()) ;
            M.Wait(200);

            M.Swing.setPosition(Materials.SwingBottomPos);
            M.Wait(50);
            M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
            M.Wait(150);
            M.Swing.setPosition(Materials.SwingInsidePos);
            M.Wait(400);
            LBT.SetAction("Reset");
            LBT.start();
        }

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class LowerBackgroundThread extends Thread {
        String Action;

        public void SetAction(String Action) {
            this.Action = Action;
        }

        public void run() {
            switch (Action) {
                case "Throw":
                    M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                    M.Wait(150);
                    M.SetPaw(Materials.PawFoldPos, 0);
                    M.Wait(100);
                    break;
                case "Reset":
                    M.SetPaw(Materials.PawThrowPos, 0);
                    M.Wait(300);
                    M.NeedToResetExtender = true;
                    while (M.NeedToResetExtender && !isStopRequested()) ;
                    break;
                case "InitOpenCV":
                    M.InitOpenCV(hardwareMap, Red);
                    break;
            }
        }
    }

    class UpperBackgroundThread extends Thread {
        private String Action;

        public void SetAction(String Action) {
            this.Action = Action;
        }

        public void run() {
            switch (Action) {
                case "ReturnFirst":
                    M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
                    M.Wait(150);
                    M.SetTargetLiftState(1);
                    M.Elbow.setPosition(Materials.ElbowWallPreparePos);
                    M.Wrist.setPosition(Materials.WristWallPos);
                    while (M.LiftPosError() < -10 && !isStopRequested()) {
                        RightFastAndFuriousCamera.this.sleep(10);
                    }
                    break;
                case "Reset":
                    M.NeedToResetLift = true;
                    while (M.NeedToResetLift && !isStopRequested()) ;
                    break;
                case "PrepareClip":
                    M.Elbow.setPosition(Materials.ElbowClippingPos);
                    M.Wrist.setPosition(Materials.WristClippingPos);
                    M.Wait(200);
                    M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
                    break;
                case "Return":
                    M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
                    M.Wait(150);
                    M.SetTargetLiftState(1);
                    M.Elbow.setPosition(Materials.ElbowWallPreparePos);
                    M.Wrist.setPosition(Materials.WristWallPos);
                    while (M.LiftPosError() < -10 && !isStopRequested()) {
                        RightFastAndFuriousCamera.this.sleep(10);
                    }
                    M.NeedToResetLift = true;
                    while (M.NeedToResetLift && !isStopRequested()) ;
                    break;

            }
        }
    }

    class DrivingThread extends Thread {
        public void run() {
            Vector2D LastXYErrorVec = new Vector2D();
            double LastAngleError = 0;
            while (!M.StopRequested) {
                Vector2D XYErrorVec = new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(), TargetY - M.Drive.getPoseEstimate().getY());
                boolean LocalWallPushing = WallPushing;
                Vector2D SFPowerVec = (LocalWallPushing ? new Vector2D(XYErrorVec.x * TranslationalKp +
                        (XYErrorVec.x - LastXYErrorVec.x) * TranslationalKd, -WallPushingPower) :
                        XYErrorVec.getMultiplied(TranslationalKp).getAdded(XYErrorVec.getSubtracted(LastXYErrorVec).getMultiplied(TranslationalKd)))
                        .getRotatedBy(-M.Drive.getPoseEstimate().getHeading());
                if (SFPowerVec.getLength() > 1) SFPowerVec.normalize();
                LastXYErrorVec.set(XYErrorVec);

                double AngleError = M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()));
                M.Drive.setWeightedDrivePower(new Pose2d(SFPowerVec.x, SFPowerVec.y,
                        M.Limit(AngleError * TurnKp + (AngleError - LastAngleError) * TurnKd, 1)
                ).times(LocalWallPushing ? 1 : Math.min(1, AccelTimer.seconds() * AccelKp)));
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
            M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
            M.Turret.setPosition(Materials.TurretStartPos);
            M.Elbow.setPosition(Materials.ElbowClippingPos);
            M.Wrist.setPosition(Materials.WristClippingPos);
            M.UpperClaw.setPosition(Materials.UpperClawClosedPos);

            M.ExtenderResetTimer.reset();
            M.LiftResetTimer.reset();
            while (!isStopRequested()) {
                M.Extender.setPower(M.NeedToResetExtender ? Materials.ExtenderResetPower : M.ExtenderToPosPower());
                if (M.NeedToResetExtender && M.ExtenderDownEnd.isPressed()) {
                    if (M.ExtenderResetTimer.milliseconds() > 200) {
                        M.ExtenderDownPos = M.Extender.getCurrentPosition();
                        M.TargetExtenderPos = 0;
                        M.ExtenderResetTimer.reset();
                        M.NeedToResetExtender = false;
                    }
                } else M.ExtenderResetTimer.reset();

                M.LiftUpdate();

                telemetry.addData("NeedToResetExtender", M.NeedToResetExtender);
                telemetry.addData("NeedToResetLift", M.NeedToResetLift);
                telemetry.update();
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
            M.SetLiftPower(0);
        }
    }
}
