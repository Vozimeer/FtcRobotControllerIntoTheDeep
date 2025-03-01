package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServosSetting extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);

        while (!isStarted() && !isStopRequested()) ;

        while (!isStopRequested()) {
//            M.Swing.setPosition(Materials.SwingTransferPos);
//            M.SetPaw(Materials.PawTransferPos, 0);
//            M.LowerClaw.setPosition(gamepad1.a ? Materials.LowerClawHardPos : Materials.LowerClawMidPos);
//            M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
//            M.Elbow.setPosition(Materials.ElbowTransferPos);
//            M.Wrist.setPosition(Materials.WristTransferPos);
//            M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);

            M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
            M.Turret.setPosition(Materials.TurretStartPos);
            M.Elbow.setPosition(gamepad1.a ? Materials.ElbowWallPos : Materials.ElbowWallPreparePos);
            M.Wrist.setPosition(Materials.WristWallPos);
//
//            M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
//            M.Turret.setPosition(Materials.TurretStartPos);
//            M.Elbow.setPosition(Materials.ElbowClippingPos);
//            M.Wrist.setPosition(Materials.WristClippingPos);
//            M.UpperClaw.setPosition(gamepad1.b ? Materials.UpperClawOpenedPos : Materials.UpperClawClosedPos);
        }
    }
}
