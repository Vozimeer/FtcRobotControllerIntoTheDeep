package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Hang extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);

        while (!isStarted() && !isStopRequested()) ;

        while (!isStopRequested()) {
            M.Extender.setPower(-gamepad1.left_stick_y);
            M.SetLiftPower(-gamepad1.right_stick_y);
        }
        M.Extender.setPower(0);
        M.SetLiftPower(0);
    }
}
