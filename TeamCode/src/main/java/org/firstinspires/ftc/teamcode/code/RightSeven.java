package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class RightSeven extends LinearOpMode {
    public static double TranslationalKp = 0.22, TranslationalKd = 1.4, TranslationalAccuracy = 2, TranslationalAccuracyToWall = 8,
            TurnKp = 0.026, TurnKd = 0.08, TurnAccuracy = 4, ExtenderCorrectionMultiply = 0.5, ExtenderAccuracy = 25,
            SampleXC = 0.8, SampleXKp = 0.041, SampleYC = 130, SampleYKp = 1.76,
            FirstClipY = 34, FirstTwoSamplesY = 11, FirstSampleX = 58.6,
            ExtenderXTrigger = 25, ExtenderFirstSamplePos = 500, SecondSampleX = 68, ExtenderSecondSamplePos = 480, ExtenderWonderThrowPos = 200,
            ThirdSampleY = 21, ThirdSampleAngle = 62, ExtenderThirdSamplePos = 300,
            ClipPrepareY = 29, ClippingStartX = -2.4, MiniExtenderXTrigger = 20, ClipY = 34.8, CameraIntakeY = 33, WallIntakeX = 48, WallIntakeY = 1;

    Materials M = new Materials();
    LowerBackgroundThread LBT = new LowerBackgroundThread();
    UpperBackgroundThread UBT = new UpperBackgroundThread();

    double TargetX = 0, TargetY = 0, TargetAngle = 90;
    boolean WallPushing = false, Red = true;
    int LeftScanI = 1, RightScanI = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        boolean DDressed = false, DLPressed = false, DRPressed = false;
        while ((M.NeedToResetExtender || M.NeedToResetLift || !isStarted()) && !isStopRequested()) {
            if (gamepad1.dpad_down && !DDressed) {
                DDressed = true;
                Red = !Red;
            }
            if (!gamepad1.dpad_down) DDressed = false;

            if (gamepad1.dpad_left && !DLPressed && RightScanI > 0) {
                DLPressed = true;
                if (RightScanI - LeftScanI == 1 && LeftScanI > 0) {
                    LeftScanI -= 1;
                    RightScanI = 3;
                } else {
                    RightScanI -= 1;
                }
            }
            if (!gamepad1.dpad_left) DLPressed = false;

            if (gamepad1.dpad_right && !DRPressed && LeftScanI < 2) {
                DRPressed = true;
                if (RightScanI == 3) {
                    LeftScanI += 1;
                    RightScanI = LeftScanI + 1;
                } else {
                    RightScanI += 1;
                }
            }
            if (!gamepad1.dpad_right) DRPressed = false;
        }
        new DrivingThread().start();
        if (LeftScanI + RightScanI > 0) {
            LBT.SetAction("InitOpenCV");
            LBT.start();
        }

        TargetY = FirstClipY;
        M.SetTargetLiftState(2);
        M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
        M.Wait(800);

        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
        M.Wait(50);
        UBT.SetAction("ReturnFirst");
        UBT.start();

        TargetY = FirstTwoSamplesY;
        M.Wait(150);
        TargetX = FirstSampleX;
        while (M.Drive.getPoseEstimate().getX() < ExtenderXTrigger && !isStopRequested()) ;
        M.TargetExtenderPos = ExtenderFirstSamplePos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);

        LowerIntake();
        TargetX = SecondSampleX;
        M.NeedToResetExtender = true;
        while (M.ExtenderPos() > ExtenderWonderThrowPos && !isStopRequested()) ;
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        M.Wait(150);

        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, 0);
        while (M.NeedToResetExtender && !isStopRequested()) ;
        M.TargetExtenderPos = ExtenderSecondSamplePos;

        LowerIntake();
        M.NeedToResetExtender = true;
        while (M.ExtenderPos() > ExtenderWonderThrowPos && !isStopRequested()) ;
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        M.Wait(150);

        TargetY = ThirdSampleY;
        TargetAngle = ThirdSampleAngle;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, 90 - ThirdSampleAngle);
        while (M.NeedToResetExtender && !isStopRequested()) ;
        M.TargetExtenderPos = ExtenderThirdSamplePos;
        UBT.SetAction("Reset");
        UBT.start();

        LowerIntake();
        TargetY = 0;
        TargetAngle = 90;
        M.Wait(300);
        M.NeedToResetExtender = true;
        while (!AtPlace(TranslationalAccuracyToWall) && !isStopRequested()) ;

        boolean Took = true;
        int ExtraCycles = 0;
        for (int i = 1; i <= 4 + ExtraCycles; i++) {
            WallPushing = true;
            while ((M.NeedToResetExtender || UBT.isAlive()) && !isStopRequested()) ;
            if (Took) {
                LBT.SetAction("Throw");
                LBT.start();
                Took = false;
            }
            M.Wait(150);
            M.Elbow.setPosition(Materials.ElbowWallPos);
            M.Wait(100);
            M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
            M.Wait(150);
            M.SetTargetLiftState(2);
            TargetY = ClipPrepareY;
            M.Drive.setPoseEstimate(new Pose2d(M.Drive.getPoseEstimate().getX(), 0, M.Drive.getPoseEstimate().getHeading()));
            WallPushing = false;

            M.Wait(150);
            TargetX = ClippingStartX + i * 3;
            M.Elbow.setPosition(Materials.ElbowClippingPos);
            M.Wrist.setPosition(Materials.WristClippingPos);
            M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
            while (M.Drive.getPoseEstimate().getX() > MiniExtenderXTrigger && !isStopRequested()) ;
            M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
            TargetY = ClipY;
            M.Wait(250);

            M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
            M.Wait(50);
            if (i == LeftScanI || i == RightScanI) {
                TargetY = CameraIntakeY;
                M.Swing.setPosition(Materials.SwingPreparePos);
                M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                UBT.SetAction("ReturnFirst");
                UBT.start();
                ElapsedTime CalmDownTimer = new ElapsedTime();
                while ((CalmDownTimer.milliseconds() < 100 || UBT.isAlive()) && !isStopRequested()) {
                    if (!AtPlace(TranslationalAccuracy)) CalmDownTimer.reset();
                }

                Pose2d LocalSamplePose = M.SDP.SamplePose;
                if (LocalSamplePose != null && !LBT.isAlive()) {
                    TargetY = M.Drive.getPoseEstimate().getY();
                    TargetX = M.Drive.getPoseEstimate().getX() + SampleXC + LocalSamplePose.getX() * SampleXKp;
                    M.TargetExtenderPos = Math.min(SampleYC + LocalSamplePose.getY() * SampleYKp, Materials.ExtenderMaxPos);
                    double PawTargetAngle = Math.toDegrees(LocalSamplePose.getHeading());
                    M.SetPaw(Materials.PawFoldPos, (PawTargetAngle > 90 ? PawTargetAngle - 90 : -(90 - PawTargetAngle)));

                    LowerIntake();
                    M.TargetExtenderPos = 450;
                    M.Wait(300);
                    M.NeedToResetExtender = true;
                    Took = true;
                    ExtraCycles += 1;
                } else {
                    M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                    M.Swing.setPosition(Materials.SwingInsidePos);
                }
                UBT.SetAction("Reset");
            } else UBT.SetAction("Return");
            UBT.start();

            TargetX = WallIntakeX;
            TargetY = WallIntakeY;
            while (!AtPlace(TranslationalAccuracyToWall) && !isStopRequested()) ;
        }

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class LowerBackgroundThread extends Thread {
        private String Action;

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
                    while (M.LiftPosError() < -100 && !isStopRequested()) ;
                    break;
                case "Reset":
                    M.NeedToResetLift = true;
                    while (M.NeedToResetLift && !isStopRequested()) ;
                    break;
                case "Return":
                    M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
                    M.Wait(150);
                    M.SetTargetLiftState(1);
                    M.Elbow.setPosition(Materials.ElbowWallPreparePos);
                    M.Wrist.setPosition(Materials.WristWallPos);
                    while (M.LiftPosError() < -100 && !isStopRequested()) ;
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
            while (!isStopRequested()) {
                Vector2D XYErrorVec = new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(), TargetY - M.Drive.getPoseEstimate().getY()),
                        TranslationalPowerVec = WallPushing ? new Vector2D(XYErrorVec.x * TranslationalKp +
                                (XYErrorVec.x - LastXYErrorVec.x) * TranslationalKd, -0.4) :
                                XYErrorVec.getMultiplied(TranslationalKp)
                                        .getAdded(XYErrorVec.getSubtracted(LastXYErrorVec).getMultiplied(TranslationalKd));
                if (TranslationalPowerVec.getLength() > 1) TranslationalPowerVec.normalize();
                LastXYErrorVec.set(XYErrorVec);

                double AngleError = M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()));
                M.SetFieldOrientedDrivePower(TranslationalPowerVec, AngleError * TurnKp +
                        (AngleError - LastAngleError) * TurnKd, ExtenderCorrectionMultiply);
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
            M.Elbow.setPosition(Materials.ElbowClippingPos);
            M.Wrist.setPosition(Materials.WristClippingPos);
            M.UpperClaw.setPosition(Materials.UpperClawClosedPos);

            M.ExtenderResetTimer.reset();
            M.LiftResetTimer.reset();
            while (!isStopRequested()) {
                M.ExtenderAutoUpdate();
                M.LiftUpdate();

                telemetry.addLine(Red ? "Red" : "Blue");
                telemetry.addData("LeftScanI", LeftScanI);
                telemetry.addData("RightScanI", RightScanI);
                telemetry.addData("NeedToResetExtender", M.NeedToResetExtender);
                telemetry.addData("NeedToResetLift", M.NeedToResetLift);
                telemetry.update();

                RightSeven.this.sleep(20);
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
            M.SetLiftPower(0);
        }
    }

    boolean AtPlace(double TranslationalAccuracy) {
        return new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                TargetY - M.Drive.getPoseEstimate().getY()).getLength() < TranslationalAccuracy &&
                Math.abs(M.MinAngleError(TargetAngle -
                        Math.toDegrees(M.Drive.getPoseEstimate().getHeading()))) < TurnAccuracy;
    }

    void LowerIntake() {
        ElapsedTime CalmDownTimer = new ElapsedTime();
        while ((CalmDownTimer.milliseconds() < 100 || M.ExtenderPosError() > ExtenderAccuracy) && !isStopRequested()) {
            if (!AtPlace(TranslationalAccuracy)) CalmDownTimer.reset();
        }

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(150);
        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingInsidePos);
        M.Wait(100);
        M.SetPaw(Materials.PawThrowPos, 0);
    }
}
