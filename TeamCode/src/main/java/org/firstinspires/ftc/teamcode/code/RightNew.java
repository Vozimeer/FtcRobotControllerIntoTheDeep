package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class RightNew extends LinearOpMode {
    public static double FirstClipY = 35, FirstTwoSamplesY = 11.8, FirstSampleX = 56.7, ExtenderFirstTwoSamplesPos = 521;

    ElapsedTime CalmDownTimer = new ElapsedTime();

    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        while (!isStarted() && !isStopRequested()) ;
        new DrivingThread().start();

        M.TargetY = FirstClipY;

        M.Wait(1000);

        M.TargetY = FirstTwoSamplesY;
        M.Wait(100);
        M.TargetX = FirstSampleX;
        while (M.Drive.getPoseEstimate().getX() < 25 && !isStopRequested()) {
            RightNew.this.sleep(10);
        }
        M.TargetExtenderPos = ExtenderFirstTwoSamplesPos;
        CalmDownTimer.reset();
        while (CalmDownTimer.milliseconds() < 100 && !isStopRequested()) {
            if (!M.AtPlace() || M.ExtenderPosError() > 20) CalmDownTimer.reset();
            RightNew.this.sleep(10);
        }

        Materials.AngleStorage = Math.toDegrees(M.Drive.getPoseEstimate().getHeading());
    }

    class DrivingThread extends Thread {
        public void run() {
            M.AutoDrivingCycle();
        }
    }

    class BackgroundThread extends Thread {
        public void run() {
            M.ExtenderResetTimer.reset();
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
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
        }
    }
}
