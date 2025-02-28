package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class Right extends LinearOpMode {
    public static double TranslationalKp = 0.14, TranslationalKd = 0.7, TurnKp = 0.026, TurnKd = 0.08,
            AccelKp = 1.6, WallPushingPower = 0.4;

    ElapsedTime AccelTimer = new ElapsedTime();
    double TargetX = 0, TargetY = 0, TargetAngle = 90;
    boolean Red = true, WallPushing = false;

    Materials M = new Materials();
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
        UCT.SetAction("InitOpenCV");
        UCT.start();

        while (!isStopRequested()) ;

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class UpperChainThread extends Thread {
        private String Action;

        public void SetAction(String Action) {
            this.Action = Action;
        }

        public void run() {
            switch (Action) {
                case "InitOpenCV":
                    M.InitOpenCV(hardwareMap, Red);
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

            while (!isStopRequested()) {
                M.Extender.setPower(M.NeedToResetExtender ? M.ExtenderResetPower() : M.ExtenderToPosPower());
                M.ExtenderResetIf();

                M.LiftUpdate();

                telemetry.addLine(Red ? "Red" : "Blue");
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
