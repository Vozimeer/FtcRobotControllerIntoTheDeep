package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Right extends LinearOpMode {
    Materials M = new Materials();
    UpperChainThread UCT = new UpperChainThread();
    boolean Red = true;

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
        M.InitOpenCV(hardwareMap, Red);

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class UpperChainThread extends Thread {
        private String Action;

        public void SetAction(String Action) {
            this.Action = Action;
        }

        public void run() {
            switch (Action) {
            }
        }
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
                M.Extender.setPower(M.NeedToResetExtender ? Materials.ExtenderResetPower : M.ExtenderToPosPower());
                if (M.NeedToResetExtender && M.ExtenderDownEnd.isPressed()) {
                    M.ExtenderDownPos = M.Extender.getCurrentPosition();
                    M.TargetExtenderPos = 0;
                    M.NeedToResetExtender = false;
                }

                M.LiftUpdate();

                telemetry.addLine(Red ? "Red" : "Blue");
                telemetry.addData("Extender", !M.NeedToResetExtender);
                telemetry.addData("Lift", !M.NeedToResetLift);
                telemetry.update();
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
            M.SetLiftPower(0);
        }
    }
}
