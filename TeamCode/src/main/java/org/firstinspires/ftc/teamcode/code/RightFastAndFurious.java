package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class RightFastAndFurious extends LinearOpMode {
    public static double TranslationalKp = 0.12, TranslationalKd = 0.6, TurnKp = 0.02, TurnKd = 0.06,
            AccelKp = 1.6, WallPushingPower = 0.4,

    FirstClipY = 34, FirstSampleX = 58, FirstTwoSamplesY = 10, ExtenderFirstTwoSamplesPos = 520,
            SamplesFastThrowExtenderPos = 200, SecondSampleX = 67,
            ThirdSampleY = 16, ThirdSampleAngle = 65, ExtenderThirdSamplePos = 450, PawThirdSampleAngle = 34,
            ClipXStep = 2, ClipPrepareY = 25, ClipY = 36, WallIntakeX = 54, WallIntakePrepareY = 2;

    public static int FirstCLipMilliseconds = 1200;

    ElapsedTime AccelTimer = new ElapsedTime();
    double TargetX = 0, TargetY = 0, TargetAngle = 90;
    boolean WallPushing = false;

    Materials M = new Materials();
    LowerBackgroundThread LBT = new LowerBackgroundThread();
    UpperBackgroundThread UBT = new UpperBackgroundThread();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        while ((M.NeedToResetExtender || M.NeedToResetLift || !isStarted()) && !isStopRequested()) ;
        new DrivingThread().start();

        AccelTimer.reset();
        TargetY = FirstClipY;
        M.SetTargetLiftState(2);
        M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
        M.Wait(FirstCLipMilliseconds);

        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
        M.Wait(100);
        UBT.SetAction("ReturnFirst");
        UBT.start();

        AccelTimer.reset();
        TargetY = FirstTwoSamplesY;
        M.Wait(200);
        TargetX = FirstSampleX;
        while (M.Drive.getPoseEstimate().getX() < 30 && !isStopRequested()) ;
        M.TargetExtenderPos = ExtenderFirstTwoSamplesPos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        while ((new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 ||
                M.ExtenderPosError() > 20) && !isStopRequested()) {
            RightFastAndFurious.this.sleep(10);
        }
        M.Wait(200);

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(50);
        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingInsidePos);
        M.Wait(50);
        M.SetPaw(Materials.PawThrowPos, 0);
        M.NeedToResetExtender = true;

        AccelTimer.reset();
        TargetX = SecondSampleX;
        while (M.ExtenderPos() > SamplesFastThrowExtenderPos && !isStopRequested()) ;
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        M.Wait(150);
        while (M.NeedToResetExtender && !isStopRequested()) ;
        M.TargetExtenderPos = ExtenderFirstTwoSamplesPos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, 0);
        while ((new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 ||
                M.ExtenderPosError() > 20) && !isStopRequested()) {
            RightFastAndFurious.this.sleep(10);
        }
        M.Wait(200);

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(50);
        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingInsidePos);
        M.Wait(50);
        M.SetPaw(Materials.PawThrowPos, 0);
        M.NeedToResetExtender = true;
        while (M.ExtenderPos() > SamplesFastThrowExtenderPos && !isStopRequested()) ;
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        M.Wait(150);

        AccelTimer.reset();
        TargetY = ThirdSampleY;
        TargetAngle = ThirdSampleAngle;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, PawThirdSampleAngle);
        while (M.NeedToResetExtender && !isStopRequested()) ;
        M.TargetExtenderPos = ExtenderThirdSamplePos;
        while ((new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 ||
                M.ExtenderPosError() > 20) && !isStopRequested()) {
            RightFastAndFurious.this.sleep(10);
        }
        while (UBT.isAlive() && !isStopRequested()) ;
        UBT.SetAction("Reset");
        UBT.start();
        M.Wait(200);

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(50);
        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingInsidePos);
        M.Wait(50);
        AccelTimer.reset();
        TargetY = 0;
        TargetAngle = 90;
        M.SetPaw(Materials.PawThrowPos, 0);
        M.Wait(300);
        M.NeedToResetExtender = true;
        while (M.NeedToResetExtender && !isStopRequested()) ;

        for (int i = 1; i <= 4; i++) {
            WallPushing = true;
            M.Wait(300);
            while (UBT.isAlive() && !isStopRequested()) ;
            if (i == 1) LBT.start();
            M.Elbow.setPosition(Materials.ElbowWallPos);
            M.Wait(100);
            M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
            M.Wait(150);
            M.SetTargetLiftState(2);
            M.Wait(200);
            M.Drive.setPoseEstimate(new Pose2d(M.Drive.getPoseEstimate().getX(), 0, M.Drive.getPoseEstimate().getHeading()));
            AccelTimer.reset();
            TargetX = ClipXStep * i;
            TargetY = ClipPrepareY;
            WallPushing = false;

            UBT.SetAction("PrepareClip");
            UBT.start();
            while (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                    TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 20 && !isStopRequested()) {
                RightFastAndFurious.this.sleep(10);
            }
            TargetY = ClipY;
            M.Wait(500);
            while (UBT.isAlive() && !isStopRequested()) ;

            M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
            M.Wait(100);
            UBT.SetAction("Return");
            UBT.start();
            AccelTimer.reset();
            TargetX = WallIntakeX;
            TargetY = WallIntakePrepareY;
            while (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                    TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 && !isStopRequested()) {
                RightFastAndFurious.this.sleep(10);
            }
        }

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class LowerBackgroundThread extends Thread {
        public void run() {
            M.LowerClaw.setPosition(Materials.LowerClawMidPos);
            M.Wait(150);
            M.SetPaw(Materials.PawFoldPos, 0);
            M.Wait(100);
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
                        RightFastAndFurious.this.sleep(10);
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
                        RightFastAndFurious.this.sleep(10);
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
