package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EMERGENCY extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);

        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, 0);
        M.LowerClaw.setPosition(Materials.LowerClawMidPos);
        M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
        M.Elbow.setPosition(Materials.ElbowBasketPos);
        M.Wrist.setPosition(Materials.WristStraightPos);
        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);

        while (!isStarted() && !isStopRequested()) ;

        while (!isStopRequested()) {
            M.Drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            M.Drive.update();

            M.Extender.setPower(-gamepad2.left_stick_y);
            M.SetLiftPower(-gamepad2.right_stick_y);

            M.MiniExtender.setPosition(gamepad2.left_bumper ? Materials.MiniExtenderClippingPos : Materials.MiniExtenderTransferPos);
        }
        M.Drive.setMotorPowers(0, 0, 0, 0);
        M.Extender.setPower(0);
        M.SetLiftPower(0);
    }
}
