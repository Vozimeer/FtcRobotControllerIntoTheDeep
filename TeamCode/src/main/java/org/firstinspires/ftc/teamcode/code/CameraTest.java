package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous
public class CameraTest extends LinearOpMode {
    public static double C = 250, SampleYMultiply = 0.08, SampleYPow = 2;
    Telemetry dash;
    Materials M = new Materials();

    boolean isReset = false;
    int isTracking = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance().getTelemetry();
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

        while (!isStopRequested()) {
            M.Drive.update();

            Pose2d Local = M.SDP.SamplePose;
            if (Local != null)
                M.TargetExtenderPos = C + Math.pow(Local.getY() * SampleYMultiply, SampleYPow);

            if (M.NeedToResetExtender) M.Extender.setPower(Materials.ExtenderResetPower);
            else if (isTracking == 1) M.Extender.setPower(M.ExtenderToPosPower());

            if (isTracking == 0){
                M.NeedToResetExtender = true;

            }

            if (gamepad1.b){
                isTracking = 0;
                while(gamepad1.b){}
            }
            else if (gamepad1.y){
                isTracking = 1;
                while(gamepad1.y){}
            }

            if (gamepad1.x){
                M.Swing.setPosition(Materials.SwingBottomPos);
            }
            else {
                M.Swing.setPosition(Materials.SwingPreparePos);
            }
            if (gamepad1.dpad_right) M.Drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
            if (M.NeedToResetExtender && M.ExtenderDownEnd.isPressed()) {
                if (M.ExtenderResetTimer.milliseconds() > 200) {
                    M.ExtenderDownPos = M.Extender.getCurrentPosition();
                    M.TargetExtenderPos = 0;
                    M.ExtenderResetTimer.reset();
                    M.Swing.setPosition(Materials.SwingPreparePos);
                    sleep(200);
                    M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                    M.NeedToResetExtender = false;
                    isTracking = -1;
                }
            } else M.ExtenderResetTimer.reset();

            dash.addData("Хэ", M.Drive.getPoseEstimate().getX());
            dash.addData("SampleZone", M.SDP.SampleZone);
            dash.addData("Extender", M.ExtenderPos());
            dash.addData("SamplePose", M.SDP.SamplePose);
            dash.update();
            telemetry.addData("Extender", M.ExtenderPos());
            telemetry.addData("SamplePose", M.SDP.SamplePose);
            telemetry.update();
        }
    }
}
