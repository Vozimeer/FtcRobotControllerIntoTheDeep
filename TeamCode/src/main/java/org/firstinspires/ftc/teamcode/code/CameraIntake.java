package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
public class CameraIntake extends LinearOpMode {
    public static double TranslationalKp = 0.2, TranslationalKd = 1.4,
            TurnKp = 0.028, TurnKd = 0.08, ExtenderCorrectionMultiply = 0.5,
            SampleXC = 4.8, SampleXKp = 0.078, SampleYC = 345, SampleYKp = 1.66;

    Materials M = new Materials();

    double TargetX = 0;
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
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.Wait(200);
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        while (!gamepad1.dpad_down && !isStopRequested()) ;

        Pose2d LocalSamplePose = M.SDP.SamplePose;
        if (LocalSamplePose != null) {
            TargetX = M.Drive.getPoseEstimate().getX() + SampleXC + LocalSamplePose.getX() * SampleXKp;
            M.TargetExtenderPos = Math.min(SampleYC + LocalSamplePose.getY() * SampleYKp, Materials.ExtenderMaxPos);
            double PawTargetAngle = Math.toDegrees(LocalSamplePose.getHeading());
            M.SetPaw(Materials.PawFoldPos, (PawTargetAngle > 90 ? PawTargetAngle - 90 : -(90 - PawTargetAngle)));
            while (gamepad1.dpad_down && !isStopRequested()) ;
            while (!gamepad1.dpad_down && !isStopRequested()) ;
        }
    }

    class DrivingThread extends Thread {
        public void run() {
            Vector2D LastXYErrorVec = new Vector2D();
            double LastAngleError = 0;
            while (!isStopRequested()) {
                Vector2D XYErrorVec = new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(), -M.Drive.getPoseEstimate().getY()),
                        TranslationalPowerVec = XYErrorVec.getMultiplied(TranslationalKp)
                                .getAdded(XYErrorVec.getSubtracted(LastXYErrorVec).getMultiplied(TranslationalKd));
                if (TranslationalPowerVec.getLength() > 1) TranslationalPowerVec.normalize();
                LastXYErrorVec.set(XYErrorVec);

                double AngleError = M.MinAngleError(90 - Math.toDegrees(M.Drive.getPoseEstimate().getHeading()));
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
            M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
            M.Elbow.setPosition(Materials.ElbowWallPreparePos);
            M.Wrist.setPosition(Materials.WristWallPos);
            M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);

            M.ExtenderResetTimer.reset();
            M.LiftResetTimer.reset();
            while (!isStopRequested()) {
                M.ExtenderAutoUpdate();
                M.LiftUpdate();

                telemetry.addLine(Red ? "Red" : "Blue");
                telemetry.addData("NeedToResetExtender", M.NeedToResetExtender);
                telemetry.addData("NeedToResetLift", M.NeedToResetLift);
                telemetry.update();

                CameraIntake.this.sleep(20);
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
            M.SetLiftPower(0);
        }
    }
}
