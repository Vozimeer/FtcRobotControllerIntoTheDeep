package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class Right extends LinearOpMode {
    public static double TranslationalKp = 0.12, TranslationalKd = 0.6, TurnKp = 0.02, TurnKd = 0.06,
            AccelKp = 1.6, WallPushingPower = 0.4,

    FirstClipY = 35, FirstSampleX = 57, FirstSampleY = 10, ExtenderFirstPos = 520,
            SecondSampleX = 67, ThirdSampleY = 16, ThirdSampleAngle = 65, ExtenderThirdPos = 430, PawThirdAngle = 34,
            SecondClipX = 1, WallIntakeX = 54;

    public static int FirstCLipMilliseconds = 1200;

    ElapsedTime AccelTimer = new ElapsedTime();
    double TargetX = 0, TargetY = 0, TargetAngle = 90;
    boolean Red = true, WallPushing = false;

    Materials M = new Materials();
    LowerChainThread LCT = new LowerChainThread();
    UpperChainThread UCT = new UpperChainThread();

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
        LCT.SetAction("InitOpenCV");
        LCT.start();

        // To first clip
        AccelTimer.reset();
        TargetY = FirstClipY;
        M.SetTargetLiftState(2);
        M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
        M.Wait(FirstCLipMilliseconds);

        // First clip
        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
        M.Wait(100);
        UCT.SetAction("ReturnFirst");
        UCT.start();

        // To first sample
        AccelTimer.reset();
        TargetY = FirstSampleY;
        M.Wait(200);
        TargetX = FirstSampleX;
        while ((M.Drive.getPoseEstimate().getX() < 30 ||
                LCT.isAlive()) && !isStopRequested()) ;
        LCT.ExtenderPos = ExtenderFirstPos;
        LCT.SetAction("Prepare");
        LCT.start();
        while (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 && !isStopRequested()) {
            Right.this.sleep(10);
        }
        M.Wait(300);
        while (LCT.isAlive() && !isStopRequested()) ;

        // Throw and to second
        LCT.SetAction("FastThrow");
        LCT.start();
        M.Wait(300);
        AccelTimer.reset();
        TargetX = SecondSampleX;
        while (LCT.isAlive() && !isStopRequested()) ;

        // Throw second
        LCT.SetAction("Prepare");
        LCT.start();
        while (LCT.isAlive() && !isStopRequested()) ;
        LCT.SetAction("FastThrow");
        LCT.start();
        while (LCT.isAlive() && !isStopRequested()) ;

        AccelTimer.reset();
        TargetY = ThirdSampleY;
        TargetAngle = ThirdSampleAngle;
        M.Wait(600);

        LCT.ExtenderPos = ExtenderThirdPos;
        LCT.PawAngle = PawThirdAngle;
        LCT.SetAction("Prepare");
        LCT.start();
        while (LCT.isAlive() && !isStopRequested()) ;

        LCT.SetAction("PrepareThrow");
        LCT.start();
        M.Wait(300);
        AccelTimer.reset();
        TargetY = 0;
        TargetAngle = 90;
        M.Wait(400);
        UCT.SetAction("Reset");
        UCT.start();
        for (int i = 1; i <= 4; i++) {
            WallPushing = true;
            M.Wait(300);
            while ((LCT.isAlive() || UCT.isAlive()) && !isStopRequested()) ;
            LCT.SetAction("Throw");
            LCT.start();
            UCT.SetAction("WallIntake");
            UCT.start();
            M.Wait(300);
            M.Drive.setPoseEstimate(new Pose2d(M.Drive.getPoseEstimate().getX(), 0, M.Drive.getPoseEstimate().getHeading()));
            WallPushing = false;

            AccelTimer.reset();
            TargetX = SecondClipX * i;
            TargetY = FirstClipY;
            while (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                    TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 10 && !isStopRequested()) {
                Right.this.sleep(10);
            }
            M.Wait(500);

            M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
            M.Wait(100);
            UCT.SetAction("Return");
            UCT.start();
            AccelTimer.reset();
            TargetX = WallIntakeX;
            TargetY = 3;
            while (new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                    TargetY - M.Drive.getPoseEstimate().getY()).getLength() > 2 && !isStopRequested()) {
                Right.this.sleep(10);
            }
            while (LCT.isAlive() && !isStopRequested()) ;
        }

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class LowerChainThread extends Thread {
        private String Action;
        private double ExtenderPos = 0, PawAngle = 0;

        public void SetAction(String Action) {
            this.Action = Action;
        }

        public void run() {
            switch (Action) {
                case "Prepare":
                    M.TargetExtenderPos = ExtenderPos;
                    M.Swing.setPosition(Materials.SwingPreparePos);
                    M.Wait(150);
                    M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                    M.Wait(50);
                    M.SetPaw(Materials.PawFoldPos, PawAngle);
                    while (M.ExtenderPosError() > 20 && !isStopRequested()) {
                        Right.this.sleep(10);
                    }
                    M.Wait(200);
                    break;
                case "FastThrow":
                    M.Swing.setPosition(Materials.SwingBottomPos);
                    M.Wait(50);
                    M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
                    M.Wait(150);
                    M.Swing.setPosition(Materials.SwingInsidePos);
                    M.Wait(400);
                    M.SetPaw(Materials.PawThrowPos, 0);
                    M.NeedToResetExtender = true;
                    while (M.ExtenderPos() > 60 && !isStopRequested()) {
                        Right.this.sleep(10);
                    }
                    M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                    M.Wait(150);
                    M.SetPaw(Materials.PawFoldPos, 0);
                    M.Wait(100);
                    while (M.NeedToResetExtender && !isStopRequested()) ;
                    break;
                case "PrepareThrow":
                    M.Swing.setPosition(Materials.SwingBottomPos);
                    M.Wait(50);
                    M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
                    M.Wait(150);
                    M.Swing.setPosition(Materials.SwingInsidePos);
                    M.Wait(400);
                    M.SetPaw(Materials.PawThrowPos, 0);
                    M.NeedToResetExtender = true;
                    while (M.NeedToResetExtender && !isStopRequested()) ;
                    break;
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

    class UpperChainThread extends Thread {
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
                case "WallIntake":
                    M.Elbow.setPosition(Materials.ElbowWallPos);
                    M.Wait(100);
                    M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
                    M.Wait(150);
                    M.SetTargetLiftState(2);
                    M.Wait(200);
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

                telemetry.addLine(Red ? "Red" : "Blue");
                telemetry.addData("Extender", M.ExtenderPos());
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
