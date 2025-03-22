package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class Left extends LinearOpMode {
    public static double TranslationalKp = 0.2, TranslationalKd = 1.4, TranslationalAccuracy = 2,
            TurnKp = 0.026, TurnKd = 0.12, TurnAccuracy = 4, ExtenderCorrectionMultiply = 0.6, ExtenderAccuracy = 20,
            FirstBasketX = -16, FirstBasketY = 4, FirstSampleX = -8.3, SamplesY = 15, ExtenderFirstSamplePos = 420,
            SecondBasketX = -16, SecondBasketY = 4, SecondSampleX = -19.6, ExtenderSecondSamplePos = 420,
            ThirdBasketX = -16, ThirdBasketY = 4, ThirdSampleX = -19, ThirdSampleAngle = 112, ExtenderThirdSamplePos = 500,
            FourthBasketX = -16, FourthBasketY = 4.4, ParkY = 52, ParkX = 18.6, ElbowParkPos = 0.7;

    Materials M = new Materials();
    ReturnUpperBodyThread RUBT = new ReturnUpperBodyThread();

    double TargetX = 0, TargetY = 0, TargetAngle = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        while ((M.NeedToResetExtender || M.NeedToResetLift || !isStarted()) && !isStopRequested()) ;
        new DrivingThread().start();

        TargetX = FirstBasketX;
        TargetY = FirstBasketY;
        TargetAngle = 45;
        M.SetTargetLiftState(3);
        M.Elbow.setPosition(Materials.ElbowVerticalPos);
        Dunk();

        TargetX = FirstSampleX;
        TargetY = SamplesY;
        TargetAngle = 90;
        M.TargetExtenderPos = ExtenderFirstSamplePos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        LowerIntake();

        TargetX = SecondBasketX;
        TargetY = SecondBasketY;
        TargetAngle = 45;
        while (RUBT.isAlive() && !isStopRequested()) ;
        Transfer();
        Dunk();

        TargetX = SecondSampleX;
        TargetY = SamplesY;
        TargetAngle = 90;
        M.TargetExtenderPos = ExtenderSecondSamplePos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, 0);
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        LowerIntake();

        TargetX = ThirdBasketX;
        TargetY = ThirdBasketY;
        TargetAngle = 45;
        while (RUBT.isAlive() && !isStopRequested()) ;
        Transfer();
        Dunk();

        TargetX = ThirdSampleX;
        TargetY = SamplesY;
        TargetAngle = ThirdSampleAngle;
        M.TargetExtenderPos = ExtenderThirdSamplePos;
        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, -(ThirdSampleAngle - 90));
        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
        LowerIntake();
        M.SetPaw(Materials.PawFoldPos, 0);

        TargetX = FourthBasketX;
        TargetY = FourthBasketY;
        TargetAngle = 45;
        while (RUBT.isAlive() && !isStopRequested()) ;
        Transfer();
        M.Swing.setPosition(Materials.SwingInsidePos);
        M.SetPaw(Materials.PawFoldPos, 0);
        Dunk();

        TargetX = 0;
        TargetY = ParkY;
        TargetAngle = 0;
        while ((!AtPlace() || RUBT.isAlive()) && !isStopRequested()) ;

        TargetX = ParkX;
        M.SetTargetLiftState(1);
        M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
        M.Elbow.setPosition(ElbowParkPos);
        M.Wrist.setPosition(Materials.WristStraightPos);
        while (!isStopRequested()) ;

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class ReturnUpperBodyThread extends Thread {
        public void run() {
            M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
            M.Elbow.setPosition(Materials.ElbowVerticalPos);
            M.SetTargetLiftState(1);
            while (M.LiftPosError() < -100 && !isStopRequested()) ;
            M.Elbow.setPosition(Materials.ElbowWallPreparePos);
            M.Wrist.setPosition(Materials.WristWallPos);
            M.NeedToResetLift = true;
            while (M.NeedToResetLift && !isStopRequested()) ;
        }
    }

    class DrivingThread extends Thread {
        public void run() {
            Vector2D LastXYErrorVec = new Vector2D();
            double LastAngleError = 0;
            while (!isStopRequested()) {
                Vector2D XYErrorVec = new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(), TargetY - M.Drive.getPoseEstimate().getY()),
                        TranslationalPowerVec = XYErrorVec.getMultiplied(TranslationalKp)
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
            M.Elbow.setPosition(Materials.ElbowTransferPos);
            M.Wrist.setPosition(Materials.WristStraightPos);
            M.UpperClaw.setPosition(Materials.UpperClawClosedPos);

            M.ExtenderResetTimer.reset();
            M.LiftResetTimer.reset();
            while (!isStopRequested()) {
                M.ExtenderAutoUpdate();
                M.LiftUpdate();

                telemetry.addData("NeedToResetExtender", M.NeedToResetExtender);
                telemetry.addData("NeedToResetLift", M.NeedToResetLift);
                telemetry.update();

                Left.this.sleep(20);
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
            M.SetLiftPower(0);
        }
    }

    boolean AtPlace() {
        return new Vector2D(TargetX - M.Drive.getPoseEstimate().getX(),
                TargetY - M.Drive.getPoseEstimate().getY()).getLength() < TranslationalAccuracy &&
                Math.abs(M.MinAngleError(TargetAngle -
                        Math.toDegrees(M.Drive.getPoseEstimate().getHeading()))) < TurnAccuracy;
    }

    void Dunk() {
        ElapsedTime CalmDownTimer = new ElapsedTime();
        while ((CalmDownTimer.milliseconds() < 100 || M.LiftPosError() > 250) && !isStopRequested()) {
            if (!AtPlace()) CalmDownTimer.reset();
        }

        M.Elbow.setPosition(Materials.ElbowBasketPos);
        M.Wait(300);
        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
        M.Wait(200);
        RUBT.start();
    }

    void LowerIntake() {
        ElapsedTime CalmDownTimer = new ElapsedTime();
        while ((CalmDownTimer.milliseconds() < 200 || M.ExtenderPosError() > ExtenderAccuracy) && !isStopRequested()) {
            if (!AtPlace()) CalmDownTimer.reset();
        }

        M.Swing.setPosition(Materials.SwingBottomPos);
        M.Wait(150);
        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);
        M.Swing.setPosition(Materials.SwingTransferPos);
        M.Wait(100);
    }

    void Transfer() {
        M.NeedToResetExtender = true;
        M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
        M.Elbow.setPosition(Materials.ElbowTransferPreparePos);
        M.Wrist.setPosition(Materials.WristTransferPos);
        while (M.ExtenderPos() > 100 && !isStopRequested()) ;
        M.LowerClaw.setPosition(Materials.LowerClawHardPos);
        M.Wait(50);
        M.SetPaw(Materials.PawTransferPos, 0);
        M.Wait(300);
        M.Elbow.setPosition(Materials.ElbowTransferPos);
        M.Wait(100);

        M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
        M.Wait(100);
        M.LowerClaw.setPosition(Materials.LowerClawMidPos);
        M.Wait(100);

        M.SetTargetLiftState(3);
        M.Wait(100);
        M.Elbow.setPosition(Materials.ElbowVerticalPos);
        M.Wrist.setPosition(Materials.WristStraightPos);
    }
}
