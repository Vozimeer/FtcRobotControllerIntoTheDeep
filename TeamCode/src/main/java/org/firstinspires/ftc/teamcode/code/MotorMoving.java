package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorMoving extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Extender = hardwareMap.get(DcMotorEx.class, "Extender");
        M.Extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        M.Extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        M.Extender.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready!");
            telemetry.update();
        }
        telemetry.addLine();
        telemetry.update();

        while (!isStopRequested()) {
            M.Extender.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
        M.Extender.setPower(0);
    }
}
