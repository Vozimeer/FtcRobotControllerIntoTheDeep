package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
public class CameraTest extends LinearOpMode {
    public static double C = 166, SampleYMultiply = 0.0081, SampleYPow = 2;

    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);

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
        M.Wait(6000);

        while (!isStopRequested()) {
            Pose2d Local = M.SDP.SamplePose;
            if (Local != null) {
                M.TargetExtenderPos = C + Math.pow(Local.getY() * SampleYMultiply, SampleYPow);
            }

            M.Extender.setPower(M.NeedToResetExtender ? Materials.ExtenderResetPower : M.ExtenderToPosPower());
            if (M.NeedToResetExtender && M.ExtenderDownEnd.isPressed()) {
                if (M.ExtenderResetTimer.milliseconds() > 200) {
                    M.ExtenderDownPos = M.Extender.getCurrentPosition();
                    M.TargetExtenderPos = 0;
                    M.ExtenderResetTimer.reset();
                    M.NeedToResetExtender = false;
                }
            } else M.ExtenderResetTimer.reset();

            telemetry.addData("Extender", M.ExtenderPos());
            telemetry.addData("SamplePose", M.SDP.SamplePose);
            telemetry.update();
        }
    }
}
