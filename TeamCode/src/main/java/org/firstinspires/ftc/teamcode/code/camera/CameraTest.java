package org.firstinspires.ftc.teamcode.code.camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class CameraTest extends LinearOpMode {
    CameraMaterials CM = new CameraMaterials();

    @Override
    public void runOpMode() throws InterruptedException {
        boolean Red = true;
        CM.InitOpenCV(hardwareMap, Red);
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

        while (!isStopRequested()) {
            telemetry.addData("SamplePose", CM.SDP.SamplePose);
            telemetry.update();
        }
        CM.Webcam.closeCameraDevice();
    }
}
