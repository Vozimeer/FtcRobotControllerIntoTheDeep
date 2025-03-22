package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class Right extends LinearOpMode {
    public static double TranslationalKp = 0.2, TranslationalKd = 1.4, TranslationalAccuracy = 2, TranslationalAccuracyToWall = 6,
            TurnKp = 0.026, TurnKd = 0.08, TurnAccuracy = 4, ExtenderCorrectionMultiply = 0.6, ExtenderAccuracy = 20,
            SampleXC = 0, SampleXKp = 0.046, SampleYC = 150, SampleYKp = 1.6,
            FirstClipY = 34, FirstTwoSamplesY = 11, FirstSampleX = 58.6,
            ExtenderXTrigger = 25, ExtenderFirstSamplePos = 530, SecondSampleX = 67.1, ExtenderSecondSamplePos = 510, SecondSampleThrowY = 6,
            ThirdSampleY = 21, ThirdSampleAngle = 61, ExtenderThirdSamplePos = 310,
            ClipPrepareY = 28, ClippingStartX = -2.4, MiniExtenderXTrigger = 20, ClipY = 34.8, CameraIntakeY = 33.5, WallIntakeX = 47, WallIntakeY = 2;

    Materials M = new Materials();
    LowerBackgroundThread LBT = new LowerBackgroundThread();
    UpperBackgroundThread UBT = new UpperBackgroundThread();

    double TargetX = 0, TargetY = 0, TargetAngle = 90;
    boolean WallPushing = false, Red = true;
    int IntakeI = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        boolean APressed = false, DLPressed = false, DRPressed = false;
        while ((M.NeedToResetExtender || M.NeedToResetLift || !isStarted()) && !isStopRequested()) {
            if (gamepad1.a && !APressed) {
                APressed = true;
                Red = !Red;
            }
            if (!gamepad1.a) APressed = false;

            if (gamepad1.dpad_left && !DLPressed && IntakeI > 0) {
                DLPressed = true;
                IntakeI -= 1;
            }
            if (!gamepad1.dpad_left) DLPressed = false;

            if (gamepad1.dpad_right && !DRPressed && IntakeI < 3) {
                DRPressed = true;
                IntakeI += 1;
            }
            if (!gamepad1.dpad_right) DRPressed = false;
        }
        new DrivingThread().start();
        if (IntakeI > 0) {
            LBT.SetAction("InitOpenCV");
            LBT.start();
        }

        TargetY = FirstClipY;
        M.SetTargetLiftState(2);
        M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
        M.Wait(900);

        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
        M.Wait(100);
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
        while (M.NeedToResetExtender && !isStopRequested()) ;
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        M.Wait(150);

        M.TargetExtenderPos = ExtenderSecondSamplePos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, 0);

        LowerIntake();
        TargetY = SecondSampleThrowY;
        M.NeedToResetExtender = true;
        while ((!AtPlace(TranslationalAccuracy) || M.NeedToResetExtender) && !isStopRequested()) ;
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        TargetY = ThirdSampleY;
        TargetAngle = ThirdSampleAngle;
        M.Wait(150);

        M.TargetExtenderPos = ExtenderThirdSamplePos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, 90 - ThirdSampleAngle);
        UBT.SetAction("Reset");
        UBT.start();

        LowerIntake();
        TargetY = 0;
        TargetAngle = 90;
        M.Wait(300);
        M.NeedToResetExtender = true;
        while (!AtPlace(TranslationalAccuracyToWall) && !isStopRequested()) ;

        boolean Took = false;
        for (int i = 1; i <= (Took ? 5 : 4); i++) {
            WallPushing = true;
            while ((M.NeedToResetExtender || UBT.isAlive()) && !isStopRequested()) ;
            if (i == 1 || Took) {
                LBT.SetAction("Throw");
                LBT.start();
            }
            M.Wait(100);
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
            M.Wait(300);

            M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
            M.Wait(100);
            if (i == IntakeI) {
                TargetY = CameraIntakeY;
                UBT.SetAction("ReturnFirst");
                UBT.start();
                while (UBT.isAlive() && !isStopRequested()) ;
                ElapsedTime CalmDownTimer = new ElapsedTime();
                while (CalmDownTimer.milliseconds() < 100 && !isStopRequested()) {
                    if (!AtPlace(TranslationalAccuracy)) CalmDownTimer.reset();
                }

                Pose2d LocalSamplePose = M.SDP.SamplePose;
                M.Webcam.closeCameraDevice();
                if (LocalSamplePose != null && !LBT.isAlive()) {
                    TargetX = M.Drive.getPoseEstimate().getX() + SampleXC + LocalSamplePose.getX() * SampleXKp;
                    M.TargetExtenderPos = Math.min(SampleYC + LocalSamplePose.getY() * SampleYKp, Materials.ExtenderMaxPos);
                    M.Swing.setPosition(Materials.SwingPreparePos);
                    double PawTargetAngle = Math.toDegrees(LocalSamplePose.getHeading());
                    M.SetPaw(Materials.PawFoldPos, (PawTargetAngle > 90 ? PawTargetAngle - 90 : -(90 - PawTargetAngle)));
                    M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);

                    LowerIntake();
                    M.TargetExtenderPos = 450;
                    M.Wait(300);
                    M.NeedToResetExtender = true;
                    Took = true;
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
                                (XYErrorVec.x - LastXYErrorVec.x) * TranslationalKd, -0.3) :
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
                telemetry.addData("i", IntakeI);
                telemetry.addData("NeedToResetExtender", M.NeedToResetExtender);
                telemetry.addData("NeedToResetLift", M.NeedToResetLift);
                telemetry.update();

                Right.this.sleep(20);
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
            M.SetLiftPower(0);
        }
    }

    boolean AtPlace(double TranslationalAccuracy) {
        return new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                TargetY - M.Drive.getPoseEstimate().getY()).getLength() < TranslationalAccuracy &&
                Math.abs(M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()))) < TurnAccuracy;
    }

    void LowerIntake() {
        ElapsedTime CalmDownTimer = new ElapsedTime();
        while (CalmDownTimer.milliseconds() < 100 && !isStopRequested()) {
            if (!AtPlace(TranslationalAccuracy) || M.ExtenderPosError() > ExtenderAccuracy)
                CalmDownTimer.reset();
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
