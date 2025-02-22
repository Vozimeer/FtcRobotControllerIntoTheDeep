package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LiftCreation extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);
        new BackgroundThread().start();

        while ((M.NeedToResetLift || !isStarted()) && !isStopRequested()) ;

        boolean APressed = false, Up = false;
        while (!isStopRequested()) {
            if (gamepad1.a && !APressed) {
                APressed = true;
                if (Up) {
                    M.SetTargetLiftPos(Materials.LiftWallCheckPos);
                    M.WaitLift();
                    M.NeedToResetLift = true;
                    while (M.NeedToResetLift && !isStopRequested()) ;
                } else {
                    M.SetTargetLiftPos(Materials.LiftBasketPos);
                }
                Up = !Up;
            }
            if (!gamepad1.a) APressed = false;
        }
    }

    class BackgroundThread extends Thread {
        public void run() {
            while (!isStopRequested()) {
                M.LiftUpdate();

                telemetry.addData("NeedToReset", M.NeedToResetLift);
                telemetry.addData("PosError", M.LiftPosError());
                telemetry.update();
            }
            M.StopRequested = true;
            M.SetLiftPower(0);
        }
    }
}
