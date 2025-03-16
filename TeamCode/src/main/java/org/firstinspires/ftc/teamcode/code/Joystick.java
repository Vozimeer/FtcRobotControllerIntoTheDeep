package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class Joystick extends LinearOpMode {
    public static double ExtenderCorrectionMultiply = 1.18,
            LiftHeightCorrectionSpeed = 2.8, PawPosCorrectionSpeed = 0.0014;

    Materials M = new Materials();
    LowerThread LT = new LowerThread();
    UpperThread UT = new UpperThread();

    int LowerChainState = 0, UpperChainState = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, true);
        new BackgroundThread().start();

        while ((M.NeedToResetExtender || M.NeedToResetLift || !isStarted()) && !isStopRequested()) ;

        boolean APressed = false, XPressed = false, BPressed = false, RBPressed = false;
        while (!isStopRequested()) {
            if (gamepad1.dpad_right) M.Drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

            double Multiply = 0.3 + (gamepad1.right_trigger * 0.7);
            M.SetFieldOrientedDrivePower(new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y)
                    .getMultiplied(Multiply), -gamepad1.right_stick_x * Multiply, ExtenderCorrectionMultiply);

            if (gamepad1.a && !APressed && !LT.isAlive()) {
                if (LowerChainState == 1) {
                    if (UT.isAlive()) {
                        if (UT.Action.equals("ToWallPrepare")) {
                            APressed = true;
                            LT.SetAction("LowerIntake");
                            LT.start();
                        }
                    } else if (UpperChainState == 0) {
                        APressed = true;
                        LT.SetAction("LowerIntake");
                        LT.start();
                    }
                } else if (LowerChainState == 2 && !UT.isAlive() && UpperChainState == 0) {
                    APressed = true;
                    LT.SetAction("Transfer");
                    LT.start();
                }
            }
            if (!gamepad1.a) APressed = false;

            if (gamepad1.x && !XPressed && !UT.isAlive() && UpperChainState == 0) {
                if (LT.isAlive()) {
                    if (LT.Action.equals("ToActivatedState") || LT.Action.equals("ToDeactivatedState")) {
                        XPressed = true;
                        UT.SetAction("WallIntake");
                        UT.start();
                    }
                } else {
                    XPressed = true;
                    if (LowerChainState == 2) {
                        UT.SetAction("UpForThrow");
                        UT.start();
                        LT.SetAction("PrepareThrow");
                        LT.start();
                    } else {
                        if (LowerChainState == 3) {
                            LT.SetAction("ToActivatedState");
                            LT.start();
                        }
                        UT.SetAction("WallIntake");
                        UT.start();
                    }
                }
            }
            if (!gamepad1.x) XPressed = false;

            if (gamepad1.b && !BPressed) {
                if (!LT.isAlive() && LowerChainState > 1) {
                    BPressed = true;
                    LT.SetAction("ToActivatedState");
                    LT.start();
                } else if (!UT.isAlive()) {
                    if (UpperChainState == 2) {
                        BPressed = true;
                        UT.SetAction("BasketOuttake");
                        UT.start();
                    } else if (UpperChainState > 0) {
                        BPressed = true;
                        UT.SetAction("ToWallPrepare");
                        UT.start();
                    }
                }
            }
            if (!gamepad1.b) BPressed = false;

            if (gamepad1.right_bumper && !RBPressed && !LT.isAlive()) {
                RBPressed = true;
                LT.SetAction(LowerChainState == 0 ? "ToActivatedState" : "ToDeactivatedState");
                LT.start();
            }
            if (!gamepad1.right_bumper) RBPressed = false;
        }
        M.Drive.setMotorPowers(0, 0, 0, 0);
    }

    boolean ExtenderActive = false;
    int SwingState = 0, PawState = 0;

    class LowerThread extends Thread {
        private String Action;

        public void SetAction(String Action) {
            this.Action = Action;
        }

        public void run() {
            switch (Action) {
                case "ToActivatedState":
                    if (LowerChainState == 2) {
                        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                        PawState = -1;
                        SwingState = -1;
                    } else {
                        if (LowerChainState == 3) {
                            M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                            M.Wait(150);
                            PawState = 0;
                            M.Wait(100);
                        }
                        ExtenderActive = true;
                        SwingState = -1;
                        M.Wait(200);
                        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                        PawState = -1;
                    }
                    LowerChainState = 1;
                    break;
                case "ToDeactivatedState":
                    if (LowerChainState == 3) {
                        M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                        M.Wait(150);
                        PawState = 0;
                        M.Wait(100);
                    } else {
                        ExtenderActive = false;
                        if (LowerChainState == 2) {
                            M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                            M.Wait(100);
                            M.NeedToResetExtender = true;
                            SwingState = 0;
                        } else {
                            M.NeedToResetExtender = true;
                            SwingState = 0;
                            M.Wait(100);
                            PawState = 0;
                            M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                        }
                        while (M.NeedToResetExtender && !isStopRequested()) ;
                    }
                    LowerChainState = 0;
                    break;
                case "LowerIntake":
                    M.LowerClaw.setPosition(Materials.LowerClawSoftPos);
                    M.Wait(150);
                    SwingState = 2;
                    M.Wait(50);
                    PawState = 0;
                    LowerChainState = 2;
                    break;
                case "PrepareThrow":
                    ExtenderActive = false;
                    SwingState = 0;
                    M.NeedToResetExtender = true;
                    M.Wait(200);
                    PawState = 1;
                    M.Wait(100);
                    while (M.NeedToResetExtender && !isStopRequested()) ;
                    LowerChainState = 3;
                    break;
                case "Transfer":
                    ExtenderActive = false;
                    M.NeedToResetExtender = true;
                    M.MiniExtender.setPosition(Materials.MiniExtenderTransferPos);
                    M.Elbow.setPosition(Materials.ElbowTransferPreparePos);
                    M.Wrist.setPosition(Materials.WristTransferPos);
                    SwingState = 1;
                    while (M.ExtenderPos() > 100 && !isStopRequested()) {
                        Joystick.this.sleep(10);
                    }
                    M.LowerClaw.setPosition(Materials.LowerClawHardPos);
                    PawState = 2;
                    M.Wait(300);
                    M.Elbow.setPosition(Materials.ElbowTransferPos);
                    M.Wait(150);

                    M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
                    M.Wait(100);
                    M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                    M.Wait(100);

                    M.SetTargetLiftState(3);
                    M.Wait(100);
                    ExtenderActive = true;
                    SwingState = -1;
                    PawState = -1;
                    M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                    LowerChainState = 1;
                    M.Elbow.setPosition(Materials.ElbowBasketPos);
                    M.Wrist.setPosition(Materials.WristStraightPos);
                    UpperChainState = 2;
                    break;
            }
        }
    }

    class UpperThread extends Thread {
        private String Action;

        public void SetAction(String Action) {
            this.Action = Action;
        }

        public void run() {
            switch (Action) {
                case "UpForThrow":
                    M.SetTargetLiftState(1);
                    M.Wait(300);
                    M.NeedToResetLift = true;
                    while (M.NeedToResetLift && !isStopRequested()) ;
                    break;
                case "BasketOuttake":
                    M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
                    UpperChainState = 3;
                    break;
                case "ToWallPrepare":
                    if (UpperChainState == 1) {
                        M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);
                        M.Wait(100);
                        M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
                        M.Wait(150);
                        M.SetTargetLiftState(1);
                    } else {
                        M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
                        M.Wait(100);
                        M.SetTargetLiftState(1);
                        M.Wait(200);
                    }
                    M.Elbow.setPosition(Materials.ElbowWallPreparePos);
                    M.Wrist.setPosition(Materials.WristWallPos);
                    while (M.LiftPosError() < -10 && !isStopRequested()) {
                        Joystick.this.sleep(10);
                    }
                    M.NeedToResetLift = true;
                    while (M.NeedToResetLift && !isStopRequested()) ;
                    UpperChainState = 0;
                    break;
                case "WallIntake":
                    M.Elbow.setPosition(Materials.ElbowWallPos);
                    M.Wait(100);
                    M.UpperClaw.setPosition(Materials.UpperClawClosedPos);
                    M.Wait(150);
                    M.SetTargetLiftState(2);
                    M.Wait(100);
                    M.Elbow.setPosition(Materials.ElbowClippingPos);
                    M.Wrist.setPosition(Materials.WristClippingPos);
                    M.Wait(200);
                    M.MiniExtender.setPosition(Materials.MiniExtenderClippingPos);
                    UpperChainState = 1;
                    break;
            }
        }
    }

    class BackgroundThread extends Thread {
        public void run() {
            M.LowerClaw.setPosition(Materials.LowerClawMidPos);
            M.MiniExtender.setPosition(Materials.MiniExtenderWallPos);
            M.Elbow.setPosition(Materials.ElbowWallPreparePos);
            M.Wrist.setPosition(Materials.WristWallPos);
            M.UpperClaw.setPosition(Materials.UpperClawOpenedPos);

            double PawHeading = Math.toRadians(90), PawCurrentAngle = 0;
            boolean TriangleResetting = false;
            int ExtenderBorder = 0;
            M.ExtenderResetTimer.reset();
            M.LiftResetTimer.reset();
            while (!isStopRequested()) {
                double LeftTriggerFirst = gamepad1.left_trigger,
                        ExtenderUpr = LeftTriggerFirst == 0 ? gamepad2.right_trigger - gamepad2.left_trigger : LeftTriggerFirst;
                if (M.NeedToResetExtender) M.Extender.setPower(Materials.ExtenderResetPower);
                else if (ExtenderActive) {
                    if (gamepad1.y) {
                        TriangleResetting = true;
                        ExtenderBorder = 0;
                    }
                    if (ExtenderUpr > 0 && ExtenderBorder != 1) {
                        TriangleResetting = false;
                        if (M.ExtenderPos() >= Materials.ExtenderMaxPos) {
                            M.Extender.setPower(M.ExtenderToPosPower());
                            ExtenderBorder = 1;
                        } else {
                            M.Extender.setPower(ExtenderUpr);
                            M.TargetExtenderPos = M.ExtenderPos();
                            ExtenderBorder = 0;
                        }
                    } else if (ExtenderUpr < 0 && ExtenderBorder != -1) {
                        TriangleResetting = false;
                        if (M.ExtenderDownEnd.isPressed()) {
                            M.Extender.setPower(M.ExtenderToPosPower());
                            ExtenderBorder = -1;
                        } else {
                            M.Extender.setPower(ExtenderUpr);
                            M.TargetExtenderPos = M.ExtenderPos();
                            ExtenderBorder = 0;
                        }
                    } else
                        M.Extender.setPower(TriangleResetting ? Materials.ExtenderResetPower : M.ExtenderToPosPower());
                } else {
                    TriangleResetting = false;
                    M.Extender.setPower(M.ExtenderToPosPower());
                }
                if ((M.NeedToResetExtender || TriangleResetting) && M.ExtenderDownEnd.isPressed()) {
                    if (M.ExtenderResetTimer.milliseconds() > Materials.ExtenderResetMilliseconds) {
                        M.ExtenderDownPos = M.Extender.getCurrentPosition();
                        M.TargetExtenderPos = 0;
                        ExtenderBorder = -1;
                        M.ExtenderResetTimer.reset();
                        M.NeedToResetExtender = false;
                        TriangleResetting = false;
                    }
                } else M.ExtenderResetTimer.reset();

                M.LiftUpdate();

                int LocalSwingState = SwingState;
                M.Swing.setPosition(LocalSwingState == -1 ? gamepad1.left_bumper ? Materials.SwingBottomPos : Materials.SwingPreparePos :
                        LocalSwingState == 0 ? Materials.SwingInsidePos : LocalSwingState == 1 ? Materials.SwingTransferPos : Materials.SwingCheckPos);

                if (gamepad1.touchpad_finger_1)
                    PawHeading = new Vector2D(gamepad1.touchpad_finger_1_x, gamepad1.touchpad_finger_1_y).getAngle();
                else {
                    Vector2D RightStick2Vec = new Vector2D(gamepad2.left_stick_x, -gamepad2.left_stick_y);
                    if (RightStick2Vec.getLength() > 0.8) PawHeading = RightStick2Vec.getAngle();
                }
                int LocalPawState = PawState;
                if (LocalPawState == -1) {
                    double PawRawTargetAngle = M.MinAngleError(Math.toDegrees(PawHeading - M.Drive.getPoseEstimate().getHeading())),
                            PawTargetAngle = PawRawTargetAngle + (PawRawTargetAngle < -90 ? 180 : PawRawTargetAngle > 90 ? -180 : 0);
                    if (Math.abs(PawTargetAngle - PawCurrentAngle) <= 170)
                        PawCurrentAngle = PawTargetAngle;
                    M.SetPaw(Materials.PawFoldPos, PawCurrentAngle);
                } else {
                    PawCurrentAngle = 0;
                    M.SetPaw(LocalPawState == 0 ? Materials.PawFoldPos : LocalPawState == 1 ? Materials.PawThrowPos :
                            Materials.PawTransferPos, PawCurrentAngle);
                }

                if (gamepad2.dpad_right) {
                    if (gamepad2.a) Materials.LiftClippingPos -= LiftHeightCorrectionSpeed;
                    if (gamepad2.b) Materials.LiftClippingPos += LiftHeightCorrectionSpeed;
                    if (gamepad2.x) Materials.LiftBasketPos -= LiftHeightCorrectionSpeed;
                    if (gamepad2.y) Materials.LiftBasketPos += LiftHeightCorrectionSpeed;
                    Materials.PawStartPos -= gamepad2.right_stick_y * PawPosCorrectionSpeed;
                    Materials.PawStartRotation += gamepad2.right_stick_x * (PawPosCorrectionSpeed * 0.5);
                }

                telemetry.addData("NeedToResetExtender", M.NeedToResetExtender);
                telemetry.addData("NeedToResetLift", M.NeedToResetLift);
                telemetry.addData("LiftPosError", M.LiftPosError());
                telemetry.addData("\nLiftClippingPos", Materials.LiftClippingPos);
                telemetry.addData("LiftBasketPos", Materials.LiftBasketPos);
                telemetry.addData("PawStartPos", Materials.PawStartPos);
                telemetry.addData("PawStartRotation", Materials.PawStartRotation);
                telemetry.update();
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
            M.SetLiftPower(0);
        }
    }
}
