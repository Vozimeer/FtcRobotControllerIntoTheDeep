package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServosSetting extends LinearOpMode {
    Materials M = new Materials();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, false);

        M.Swing.setPosition(Materials.SwingCheckPos);
        M.SetPaw(Materials.PawFoldPos, 0);
        M.LowerClaw.setPosition(Materials.LowerClawMidPos);
        M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
        M.Turret.setPosition(Materials.TurretStartPos);
        M.Elbow.setPosition(Materials.ElbowWallPreparePos);
        M.Wrist.setPosition(Materials.WristWallPos);
        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);

        while (!isStarted() && !isStopRequested()) ;

        M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
        M.Wait(150);

        M.Swing.setPosition(Materials.SwingTransferPos);
        M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
        M.Elbow.setPosition(Materials.ElbowTransferPreparePos);
        M.Wrist.setPosition(Materials.WristTransferPos);
        M.Wait(200);

        M.SetPaw(Materials.PawTransferPos, 0);
        M.Wait(300);

        M.LowerClaw.setPosition(Materials.LowerClawHardPos);
        M.Elbow.setPosition(Materials.ElbowTransferPos);
        M.Wait(50);

        M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
        M.Wait(100);
        M.LowerClaw.setPosition(Materials.LowerClawMidPos);
        M.Wait(100);

        M.Swing.setPosition(Materials.SwingPreparePos);
        M.SetPaw(Materials.PawFoldPos, 0);
        M.Elbow.setPosition(Materials.ElbowBasketPos);
        M.Wrist.setPosition(Materials.WristStraightPos);
        M.Wait(250);
//        while (!isStopRequested()) {
//            while (!gamepad1.a && !isStopRequested()) ;
//
//            M.Swing.setPosition(Materials.SwingTransferPos);
//            M.SetPaw(Materials.PawTransferPos, 0);
//            M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
//            M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
//            M.Turret.setPosition(Materials.TurretStartPos);
//            M.Elbow.setPosition(Materials.ElbowTransferPos);
//            M.Wrist.setPosition(Materials.WristTransferPos);
//            M.UpperClaw.setPosition(Materials.UpperClawClosedPos);

//            M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
//            M.Turret.setPosition(Materials.TurretStartPos);
//            M.Elbow.setPosition(gamepad1.a ? Materials.ElbowWallPos : Materials.ElbowWallPreparePos);
//            M.Wrist.setPosition(Materials.WristWallPos);

//            M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
//            M.Turret.setPosition(Materials.TurretStartPos);
//            M.Elbow.setPosition(Materials.ElbowClippingPos);
//            M.Wrist.setPosition(Materials.WristClippingPos);
//            M.UpperClaw.setPosition(gamepad1.b ? Materials.UpperClawOpenedPos : Materials.UpperClawClosedPos);
//        }
    }
}
