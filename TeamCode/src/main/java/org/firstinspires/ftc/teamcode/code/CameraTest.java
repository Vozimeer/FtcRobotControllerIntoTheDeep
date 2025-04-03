package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
public class CameraTest extends LinearOpMode {
    Materials M = new Materials();

    boolean Red = true;

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);

        boolean APressed = false;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a && !APressed) {
                APressed = true;
                Red = !Red;
            }
            if (!gamepad1.a) APressed = false;
            telemetry.addLine(Red ? "Red" : "Blue");
            telemetry.update();
        }
        M.InitOpenCV(hardwareMap, Red);
        M.Wait(3000);

        while (!isStopRequested()) {
            telemetry.addLine(Red ? "Red" : "Blue");
            telemetry.addData("SamplePose", M.SDP.SamplePose);
            telemetry.addData("SampleArea", M.SDP.SampleArea);
            telemetry.update();
        }
    }
}
