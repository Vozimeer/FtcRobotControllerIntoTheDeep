package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LiftCreation extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);

        while (!isStarted() && !isStopRequested()) ;

        while (!isStopRequested()) {
            M.SetLiftPower(-gamepad1.left_stick_y);
            telemetry.addData("LeftLift", M.LeftLift.getCurrentPosition());
            telemetry.update();
        }
        M.SetLiftPower(0);
    }
}
