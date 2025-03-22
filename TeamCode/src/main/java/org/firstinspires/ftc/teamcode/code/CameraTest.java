package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class CameraTest extends LinearOpMode {
    Materials M = new Materials();

    boolean Red = true;

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        boolean APressed = false;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a && !APressed) {
                APressed = true;
                Red = !Red;
            }
            if (!gamepad1.a) APressed = false;
        }
        M.InitOpenCV(hardwareMap, Red);

        while (!isStopRequested()) ;
    }

    class BackgroundThread extends Thread {
        public void run() {
            while (!isStopRequested()) {
                telemetry.addLine(Red ? "Red" : "Blue");
                telemetry.addData("SamplePose", M.SDP.SamplePose);
                telemetry.addData("SampleArea", M.SDP.SampleArea);
                telemetry.update();
            }
        }
    }
}
