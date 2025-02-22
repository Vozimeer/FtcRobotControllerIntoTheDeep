package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class MotorMoving extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx First, Second;

        First = hardwareMap.get(DcMotorEx.class, "Extender");
        First.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        First.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        First.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Second = hardwareMap.get(DcMotorEx.class, "FL");
        Second.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Second.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Second.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready!");
            telemetry.update();
        }
        telemetry.clearAll();
        telemetry.update();

        while (!isStopRequested()) {
            First.setPower(-gamepad1.left_stick_y);
            Second.setPower(-gamepad1.right_stick_y);
            telemetry.addData("First", First.getCurrentPosition());
            telemetry.update();
        }
        First.setPower(0);
        Second.setPower(0);
    }
}
