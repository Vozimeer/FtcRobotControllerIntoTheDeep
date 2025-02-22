package org.firstinspires.ftc.teamcode.code.camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.code.Materials;

@Autonomous
public class CameraTest extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        boolean Red = true;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                Red = false;
            }
            if (gamepad1.b) {
                Red = true;
            }
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
        M.Webcam.closeCameraDevice();
    }
}
