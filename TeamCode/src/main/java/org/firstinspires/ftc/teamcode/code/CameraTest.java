package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class CameraTest extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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
        telemetry.clearAll();
        telemetry.update();
        M.InitOpenCV(hardwareMap, Red);

        M.ExtenderResetTimer.reset();
        while (!isStopRequested()) {
            if (gamepad1.dpad_right) M.Drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

            Pose2d LocalSamplePose = M.SDP.SamplePose;
            if (LocalSamplePose != null)
                M.TargetExtenderPos = Math.min(Right.SampleYC + LocalSamplePose.getY() * Right.SampleYKp, Materials.ExtenderMaxPos);

            M.Drive.update();

            M.Extender.setPower(M.NeedToResetExtender ? Materials.ExtenderResetPower : M.ExtenderToPosPower());
            if (M.NeedToResetExtender && M.ExtenderDownEnd.isPressed()) {
                if (M.ExtenderResetTimer.milliseconds() > Materials.ExtenderResetMilliseconds) {
                    M.ExtenderDownPos = M.Extender.getCurrentPosition();
                    M.TargetExtenderPos = 0;
                    M.ExtenderResetTimer.reset();
                    M.NeedToResetExtender = false;
                }
            } else M.ExtenderResetTimer.reset();

            telemetry.addData("NeedToResetExtender", M.NeedToResetExtender);
            telemetry.addData("ExtenderPos", M.ExtenderPos());
            telemetry.addData("SamplePose", M.SDP.SamplePose);
            telemetry.update();
        }
        M.Extender.setPower(0);
    }
}
