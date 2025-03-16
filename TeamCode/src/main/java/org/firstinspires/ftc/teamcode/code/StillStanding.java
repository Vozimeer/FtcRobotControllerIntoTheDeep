package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
public class StillStanding extends LinearOpMode {
    public static double FirstClipY = 35;

    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        while (!isStarted() && !isStopRequested()) ;
        new DrivingThread().start();

        M.TargetY = FirstClipY;
        while (!isStopRequested()) ;
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
