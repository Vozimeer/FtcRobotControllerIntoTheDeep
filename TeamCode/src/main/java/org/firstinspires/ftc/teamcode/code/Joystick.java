package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Joystick extends LinearOpMode {
    Materials M = new Materials();
    BackgroundThread BT = new BackgroundThread();
    ControlUpperBodyThread CUBT = new ControlUpperBodyThread();

    @Override
    public void runOpMode() throws InterruptedException {
        M.Init(hardwareMap, true);
        BT.start();

        // M.NeedToResetExtender ||
        while ((!isStarted()) && !isStopRequested()) {
            telemetry.addData("Extender", !M.NeedToResetExtender);
            telemetry.update();
        }
        telemetry.clearAll();
        telemetry.update();
        CUBT.start();

        while (!isStopRequested()) {
            if (gamepad1.dpad_right) M.Drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

            M.Drive.setWeightedDrivePower(new Pose2d(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y)
                    .rotated(-M.Drive.getPoseEstimate().getHeading()),
                    -gamepad1.right_stick_x).times(0.3 + (gamepad1.right_trigger * 0.7)));
            M.Drive.update();
        }
        M.Drive.setMotorPowers(0, 0, 0, 0);
    }

    boolean ExtenderActive = false;
    int SwingState = 0, PawState = 0;

    class ControlUpperBodyThread extends Thread {
        LowerThread LT = new LowerThread();
        int LowerChainState = 0;

        public void run() {
            LT.start();

            boolean BPressed = false;
            while (!isStopRequested()) {
                if (gamepad1.b && !BPressed && LowerChainState > 1) {
                    try {
                        SynchronizedAction("ToActivatedState");
                        BPressed = true;
                    } catch (Exception ignored) {
                    }
                }
                if (!gamepad1.b) BPressed = false;
            }
        }

        class LowerThread extends Thread {
            public void run() {
                boolean APressed = false, XPressed = false, YPressed = false, RBPressed = false;
                while (!isStopRequested()) {
                    if (gamepad1.a && !APressed && LowerChainState == 1) {
                        try {
                            SynchronizedAction("LowerIntake");
                            APressed = true;
                        } catch (Exception ignored) {
                        }
                    }
                    if (!gamepad1.a) APressed = false;

                    if (gamepad1.x && !XPressed) {
                        if (LowerChainState == 2) {
                            try {
                                SynchronizedAction("PrepareThrow");
                                XPressed = true;
                            } catch (Exception ignored) {
                            }
                        } else if (LowerChainState == 3) {
                            try {
                                SynchronizedAction("WallIntake");
                                XPressed = true;
                            } catch (Exception ignored) {
                            }
                        }
                    }
                    if (!gamepad1.x) XPressed = false;

                    if (gamepad1.y && !YPressed && (LowerChainState == 1 || LowerChainState == 2)) {
                        try {
                            SynchronizedAction("PushIn");
                            YPressed = true;
                        } catch (Exception ignored) {
                        }
                    }
                    if (!gamepad1.y) YPressed = false;

                    if (gamepad1.right_bumper && !RBPressed) {
                        try {
                            SynchronizedAction(LowerChainState == 0 ? "ToActivatedState" : "ToDeactivatedState");
                            RBPressed = true;
                        } catch (Exception ignored) {
                        }
                    }
                    if (!gamepad1.right_bumper) RBPressed = false;
                }
            }
        }

        synchronized void SynchronizedAction(String Action) {
            switch (Action) {
                case "ToActivatedState":
                    if (LowerChainState == 2) {
                        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                        PawState = -1;
                        SwingState = -1;
                    } else {
                        if (LowerChainState == 3) {
                            M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                            M.Wait(100);
                            PawState = 0;
                            M.Wait(100);
                        }
                        ExtenderActive = true;
                        SwingState = -1;
                        M.Wait(50);
                        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                        M.Wait(150);
                        PawState = -1;
                    }
                    LowerChainState = 1;
                    break;
                case "ToDeactivatedState":
                    if (LowerChainState == 3) {
                        M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                        M.Wait(100);
                        PawState = 0;
                        M.Wait(100);
                    } else {
                        ExtenderActive = false;
                        if (LowerChainState == 2) {
                            M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                            M.Wait(100);
                            SwingState = 0;
                        } else {
                            SwingState = 0;
                            M.Wait(100);
                            PawState = 0;
                            M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                        }
                        M.NeedToResetExtender = true;
                        while (M.NeedToResetExtender && !isStopRequested()) ;
                    }
                    LowerChainState = 0;
                    break;
                case "LowerIntake":
                    M.LowerClaw.setPosition(Materials.LowerClawClosedPos);
                    M.Wait(150);
                    SwingState = 2;
                    M.Wait(50);
                    PawState = 0;
                    LowerChainState = 2;
                    break;
                case "PushIn":
                    ExtenderActive = false;
                    M.NeedToResetExtender = true;
                    while (M.NeedToResetExtender && !isStopRequested()) ;
                    ExtenderActive = true;
                    break;
                case "PrepareThrow":
                    ExtenderActive = false;
                    if (M.ExtenderPos() < 350) {
                        M.TargetExtenderPos = 350;
                        M.WaitExtender();
                    }
                    SwingState = 0;
                    M.Wait(300);
                    PawState = 1;
                    M.Wait(100);
                    M.NeedToResetExtender = true;
                    while (M.NeedToResetExtender && !isStopRequested()) ;
                    LowerChainState = 3;
                    break;
                case "WallIntake":
                    if (LowerChainState == 3) {
                        M.LowerClaw.setPosition(Materials.LowerClawMidPos);
                        // Mini extender close
                        M.Wait(100);
                        PawState = 0;
                        // Close upper claw
                        M.Wait(100);
                        ExtenderActive = true;
                        SwingState = -1;
                        M.Wait(50);
                        M.LowerClaw.setPosition(Materials.LowerClawOpenedPos);
                        M.Wait(150);
                        PawState = -1;
                        LowerChainState = 1;
                    } else {
                        // Mini extender close
                        M.Wait(100);
                        // Close upper claw
                        M.Wait(300);
                    }
                    // Up
                    break;
            }
        }
    }

    class BackgroundThread extends Thread {
        public void run() {
            M.LowerClaw.setPosition(Materials.LowerClawMidPos);

            double PawHeading = Math.toRadians(90), PawCurrentAngle = 0;
            boolean ExtenderBorder = false;
            while (!isStopRequested()) {
                double LeftTrigger = gamepad1.left_trigger;
                if (M.NeedToResetExtender) M.Extender.setPower(Materials.ExtenderResetPower);
                else if (ExtenderActive && LeftTrigger > 0 && !ExtenderBorder) {
                    if (M.ExtenderPos() >= Materials.ExtenderMaxPos) {
                        M.Extender.setPower(M.ExtenderToPosPower());
                        ExtenderBorder = true;
                    } else {
                        M.Extender.setPower(LeftTrigger);
                        M.TargetExtenderPos = M.ExtenderPos();
                    }
                } else M.Extender.setPower(M.ExtenderToPosPower());
                if (M.NeedToResetExtender && M.ExtenderDownEnd.isPressed()) {
                    M.ExtenderDownPos = M.Extender.getCurrentPosition();
                    M.TargetExtenderPos = 0;
                    ExtenderBorder = false;
                    M.NeedToResetExtender = false;
                }

                int LocalSwingState = SwingState;
                M.Swing.setPosition(LocalSwingState == -1 ? gamepad1.left_bumper ? Materials.SwingBottomPos : Materials.SwingPreparePos :
                        LocalSwingState == 0 ? Materials.SwingInsidePos : LocalSwingState == 1 ? Materials.SwingTransferPos : Materials.SwingCheckPos);

                Vector2D RightStick2Vec = new Vector2D(gamepad2.right_stick_x, -gamepad2.right_stick_y);
                if (RightStick2Vec.getLength() > 0.9) PawHeading = RightStick2Vec.getAngle();
                else if (gamepad1.touchpad_finger_1)
                    PawHeading = new Vector2D(gamepad1.touchpad_finger_1_x, gamepad1.touchpad_finger_1_y).getAngle();
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
            }
            M.StopRequested = true;
            M.Extender.setPower(0);
        }
    }
}
