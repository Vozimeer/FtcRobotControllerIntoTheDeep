package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class Right extends LinearOpMode {
    public static double TranslationalKp = 0.14, TranslationalKd = 0.8, TurnKp = 0.027, TurnKd = 0.05,
            SampleXC = 0, SampleXKp = 0.039, SampleYC = 215, SampleYKp = 1.22,

    FirstClipY = 35, FirstSampleX = 56.7, FirstTwoSamplesY = 11.8, ExtenderXTrigger = 25, ExtenderFirstTwoSamplesPos = 521,
            ExtenderWonderThrowPos = 130, SecondSampleX = 66.2,
            ThirdSampleY = 18.3, ThirdSampleAngle = 65, ExtenderThirdSamplePos = 399, PawThirdSampleAngle = 34,
            ClipY = 34, MiniExtenderXTrigger = 18;

    ElapsedTime AccelTimer = new ElapsedTime(), CalmDownTimer = new ElapsedTime();
    double TargetX = 0, TargetY = 0, TargetAngle = 90;
    boolean WallPushing = false, Red = true;
    int IntakeI = 1;

    Materials M = new Materials();
    LowerBackgroundThread LBT = new LowerBackgroundThread();
    UpperBackgroundThread UBT = new UpperBackgroundThread();

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

        AccelTimer.reset();
        TargetY = FirstClipY;
        M.SetTargetLiftState(2);
        M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
        M.Wait(1000);

        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
        M.Wait(100);
        UBT.SetAction("ReturnFirst");
        UBT.start();

        AccelTimer.reset();
        TargetY = FirstTwoSamplesY;
        M.Wait(200);
        TargetX = FirstSampleX;
        while (M.Drive.getPoseEstimate().getX() < ExtenderXTrigger && !isStopRequested()) {
            Right.this.sleep(10);
        }
        M.TargetExtenderPos = ExtenderFirstTwoSamplesPos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        CalmDownTimer.reset();
        while (CalmDownTimer.milliseconds() < 150 && !isStopRequested()) {
            if (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                    TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 ||
                    Math.abs(M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()))) > 5 ||
                    Math.abs(M.ExtenderPosError()) > 20) CalmDownTimer.reset();
            Right.this.sleep(10);
        }

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(70);
        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingInsidePos);
        M.Wait(50);
        M.SetPaw(Materials.PawThrowPos, 0);
        M.NeedToResetExtender = true;

        AccelTimer.reset();
        TargetX = SecondSampleX;
        while (M.ExtenderPos() > ExtenderWonderThrowPos && !isStopRequested()) ;
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, 0);
        while (M.NeedToResetExtender && !isStopRequested()) ;
        M.TargetExtenderPos = ExtenderFirstTwoSamplesPos;
        CalmDownTimer.reset();
        while (CalmDownTimer.milliseconds() < 50 && !isStopRequested()) {
            if (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                    TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 ||
                    Math.abs(M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()))) > 5 ||
                    Math.abs(M.ExtenderPosError()) > 20) CalmDownTimer.reset();
            Right.this.sleep(10);
        }

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(70);
        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingInsidePos);
        M.Wait(50);
        M.SetPaw(Materials.PawThrowPos, 0);
        M.NeedToResetExtender = true;
        while (M.ExtenderPos() > ExtenderWonderThrowPos && !isStopRequested()) ;
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        M.Wait(150);

        AccelTimer.reset();
        TargetY = ThirdSampleY;
        TargetAngle = ThirdSampleAngle;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, PawThirdSampleAngle);
        while (M.NeedToResetExtender && !isStopRequested()) ;
        M.TargetExtenderPos = ExtenderThirdSamplePos;
        CalmDownTimer.reset();
        while (CalmDownTimer.milliseconds() < 150 && !isStopRequested()) {
            if (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                    TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 ||
                    Math.abs(M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()))) > 5 ||
                    Math.abs(M.ExtenderPosError()) > 20) CalmDownTimer.reset();
            Right.this.sleep(10);
        }
        UBT.SetAction("Reset");
        UBT.start();

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(70);
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

        boolean Took = false;
        for (int i = 1; i <= (Took ? 5 : 4); i++) {
            WallPushing = true;
            M.Wait(300);
            while ((LBT.isAlive() || UBT.isAlive()) && !isStopRequested()) ;
            if (i == 1 || Took) {
                LBT.SetAction("Throw");
                LBT.start();
            }
            M.Elbow.setPosition(Materials.ElbowWallPos);
            M.Wait(100);
            M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
            M.Wait(150);
            M.SetTargetLiftState(2);
            AccelTimer.reset();
            TargetY = ClipY;
            M.Drive.setPoseEstimate(new Pose2d(M.Drive.getPoseEstimate().getX(), 0, M.Drive.getPoseEstimate().getHeading()));
            WallPushing = false;

            M.Wait(250);
            TargetX = i * 2;
            M.Elbow.setPosition(Materials.ElbowClippingPos);
            M.Wrist.setPosition(Materials.WristClippingPos);
            M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
            while (M.Drive.getPoseEstimate().getX() > MiniExtenderXTrigger && !isStopRequested()) {
                Right.this.sleep(10);
            }
            M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
            M.Wait(300);

            M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
            M.Wait(100);
            if (i == IntakeI) {
                UBT.SetAction("ReturnFirst");
                UBT.start();
                CalmDownTimer.reset();
                while ((CalmDownTimer.milliseconds() < 50 || UBT.isAlive()) && !isStopRequested()) {
                    if (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                            TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 ||
                            Math.abs(M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()))) > 5)
                        CalmDownTimer.reset();
                    Right.this.sleep(10);
                }
                Pose2d LocalSamplePose = M.SDP.SamplePose;
                M.Webcam.closeCameraDevice();
                if (LocalSamplePose != null && !LBT.isAlive()) {
                    AccelTimer.reset();
                    TargetX = M.Drive.getPoseEstimate().getX() + SampleXC + LocalSamplePose.getX() * SampleXKp;
                    M.TargetExtenderPos = Math.min(SampleYC + LocalSamplePose.getY() * SampleYKp, Materials.ExtenderMaxPos);
                    M.Swing.setPosition(Materials.SwingPreparePos);
                    double PawTargetAngle = Math.toDegrees(LocalSamplePose.getHeading());
                    M.SetPaw(Materials.PawFoldPos, (PawTargetAngle > 90 ? PawTargetAngle - 90 : -(90 - PawTargetAngle)));
                    M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                    CalmDownTimer.reset();
                    while (CalmDownTimer.milliseconds() < 100 && !isStopRequested()) {
                        if (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                                TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 ||
                                Math.abs(M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()))) > 5 ||
                                Math.abs(M.ExtenderPosError()) > 20) CalmDownTimer.reset();
                        Right.this.sleep(10);
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
                    LBT.SetAction("Reset");
                    LBT.start();
                    Took = true;
                }
                UBT.SetAction("Reset");
                UBT.start();
            } else {
                UBT.SetAction("Return");
                UBT.start();
            }

            AccelTimer.reset();
            TargetX = 47;
            TargetY = 2;
            while (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                    TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 && !isStopRequested()) {
                Right.this.sleep(10);
            }
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
                case "Reset":
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
                        Right.this.sleep(10);
                    }
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
                    while (M.LiftPosError() < -10 && !isStopRequested()) {
                        Right.this.sleep(10);
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
            while (!isStopRequested()) {
                Vector2D XYErrorVec = new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(), TargetY - M.Drive.getPoseEstimate().getY());
                boolean LocalWallPushing = WallPushing;
                Vector2D SFPowerVec = (LocalWallPushing ? new Vector2D(XYErrorVec.x * TranslationalKp +
                        (XYErrorVec.x - LastXYErrorVec.x) * TranslationalKd, -0.4) :
                        XYErrorVec.getMultiplied(TranslationalKp).getAdded(XYErrorVec.getSubtracted(LastXYErrorVec).getMultiplied(TranslationalKd)))
                        .getRotatedBy(-M.Drive.getPoseEstimate().getHeading());
                if (SFPowerVec.getLength() > 1) SFPowerVec.normalize();
                LastXYErrorVec.set(XYErrorVec);

                double AngleError = M.MinAngleError(TargetAngle - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()));
                M.Drive.setWeightedDrivePower(new Pose2d(SFPowerVec.x, SFPowerVec.y,
                        M.Limit(AngleError * TurnKp + (AngleError - LastAngleError) * TurnKd, 1)
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
            M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
            M.Elbow.setPosition(Materials.ElbowClippingPos);
            M.Wrist.setPosition(Materials.WristClippingPos);
            M.UpperClaw.setPosition(Materials.UpperClawClosedPos);

            M.ExtenderResetTimer.reset();
            M.LiftResetTimer.reset();
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

                M.LiftUpdate();

                telemetry.addLine(Red ? "Red" : "Blue");
                telemetry.addData("i", IntakeI);
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
