package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class Left extends LinearOpMode {
    public static double TranslationalKp = 0.14, TranslationalKd = 0.7, TurnKp = 0.02, TurnKd = 0.05,

    BasketX = -16.3, BasketY = 4.2, SampleX = -7.6, SampleY = 20, ExtenderSamplePos = 300, SecondSampleX = -18.2;

    ElapsedTime AccelTimer = new ElapsedTime();
    double TargetX = 0, TargetY = 0, TargetAngle = 90;
    boolean WallPushing = false;

    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        while ((M.NeedToResetExtender || M.NeedToResetLift || !isStarted()) && !isStopRequested()) ;
        new DrivingThread().start();

        AccelTimer.reset();
        TargetX = BasketX;
        TargetY = BasketY;
        TargetAngle = 45;
        M.Wait(1000);
        M.SetTargetLiftState(3);
        M.Wait(1000);
        M.Elbow.setPosition(Materials.ElbowBasketPos);
        M.Wait(1000);

        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
        M.Wait(150);



        M.Elbow.setPosition(Materials.ElbowTransferPreparePos);
        M.Wrist.setPosition(Materials.WristTransferPos);
        M.Wait(200);
        M.SetTargetLiftState(1);
        while (M.LiftPosError() < -10 && !isStopRequested()) {
            Left.this.sleep(10);
        }
        M.NeedToResetLift = true;
        while (M.NeedToResetLift && !isStopRequested()) ;

        AccelTimer.reset();
        TargetX = SampleX;
        TargetY = SampleY;
        TargetAngle = 90;
        M.TargetExtenderPos = ExtenderSamplePos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.Wait(200);
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        M.Wait(1800);

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(70);
        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingTransferPos);
        M.Wait(500);

        AccelTimer.reset();
        TargetX = BasketX;
        TargetY = BasketY;
        TargetAngle = 45;

        M.NeedToResetExtender = true;
        while (M.ExtenderPos() > 100 && !isStopRequested()) {
            Left.this.sleep(10);
        }
        M.LowerClaw.setPosition(Materials.LowerClawHardPos);
        M.SetPaw(Materials.PawTransferPos, 0);
        M.Wait(300);
        M.Elbow.setPosition(Materials.ElbowTransferPos);
        M.Wait(150);

        M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
        M.Wait(100);
        M.LowerClaw.setPosition(Materials.LowerClawMidPos);
        M.Wait(100);

        M.SetTargetLiftState(3);
        M.Wait(2000);
        M.Swing.setPosition(Materials.SwingInsidePos);
        M.SetPaw(Materials.PawFoldPos, 0);
        M.Elbow.setPosition(Materials.ElbowBasketPos);
        M.Wrist.setPosition(Materials.WristStraightPos);
        M.Wait(500);

        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
        M.Wait(150);



        M.Elbow.setPosition(Materials.ElbowTransferPreparePos);
        M.Wrist.setPosition(Materials.WristTransferPos);
        M.Wait(200);
        M.SetTargetLiftState(1);
        while (M.LiftPosError() < -10 && !isStopRequested()) {
            Left.this.sleep(10);
        }
        M.NeedToResetLift = true;
        while (M.NeedToResetLift && !isStopRequested()) ;

        AccelTimer.reset();
        TargetX = SecondSampleX;
        TargetY = SampleY;
        TargetAngle = 90;
        M.TargetExtenderPos = ExtenderSamplePos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.Wait(200);
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        M.Wait(1800);

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(70);
        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingTransferPos);
        M.Wait(500);

        AccelTimer.reset();
        TargetX = BasketX;
        TargetY = BasketY;
        TargetAngle = 45;

        M.NeedToResetExtender = true;
        while (M.ExtenderPos() > 100 && !isStopRequested()) {
            Left.this.sleep(10);
        }
        M.LowerClaw.setPosition(Materials.LowerClawHardPos);
        M.SetPaw(Materials.PawTransferPos, 0);
        M.Wait(300);
        M.Elbow.setPosition(Materials.ElbowTransferPos);
        M.Wait(150);

        M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
        M.Wait(100);
        M.LowerClaw.setPosition(Materials.LowerClawMidPos);
        M.Wait(100);

        M.SetTargetLiftState(3);
        M.Wait(2000);
        M.Swing.setPosition(Materials.SwingInsidePos);
        M.SetPaw(Materials.PawFoldPos, 0);
        M.Elbow.setPosition(Materials.ElbowBasketPos);
        M.Wrist.setPosition(Materials.WristStraightPos);
        M.Wait(500);

        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
        M.Wait(150);



        M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
        M.Wait(150);
        M.SetTargetLiftState(1);
        M.Elbow.setPosition(Materials.ElbowWallPreparePos);
        M.Wrist.setPosition(Materials.WristWallPos);
        while (M.LiftPosError() < -10 && !isStopRequested()) {
            Left.this.sleep(10);
        }
        M.NeedToResetLift = true;
        while (M.NeedToResetLift && !isStopRequested()) ;


        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
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
            M.Elbow.setPosition(Materials.ElbowTransferPos);
            M.Wrist.setPosition(Materials.WristStraightPos);
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
