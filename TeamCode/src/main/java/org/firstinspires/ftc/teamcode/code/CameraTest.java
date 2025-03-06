package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class CameraTest extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        boolean APressed = false, Red = true;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a && !APressed) {
                APressed = true;
                Red = !Red;
            }
            if (!gamepad1.a) APressed = false;
            telemetry.addLine(Red ? "Red" : "Blue");
            telemetry.update();
        }
        telemetry.addLine();
        telemetry.update();
        M.InitOpenCV(hardwareMap, Red);

        while (!isStopRequested()) {
            telemetry.addData("SamplePose", M.SDP.SamplePose);
            telemetry.update();
        }
    }
}
