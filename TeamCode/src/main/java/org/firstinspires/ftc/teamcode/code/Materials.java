package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Materials {
    // 192.168.43.1:8080/dash

    public static double AngleStorage = 90, ExtenderKp = 0.012, ExtenderResetPower = -0.8, ExtenderMaxPos = 540,
            LiftRegularKp = 0.008, LiftMaxKp = 0.1, LiftResetPower = -0.3, LiftPushingStart = 1, LiftPushingAccel = 0.08,
            LiftWallCheckPos = 150, LiftClippingPos = 330, LiftBasketPos = 840,

    SwingInsidePos = 0.06, SwingTransferPos = 0.3, SwingCheckPos = 0.4, SwingPreparePos = 0.45, SwingBottomPos = 0.55,
            PawFoldPos = 0, PawThrowPos = 0.2, PawTransferPos = 0.33, PawStartRotation = 0.52, PawAngleMultiply = 0.0017,
            LowerClawClosedPos = 0.79, LowerClawMidPos = 0.72, LowerClawOpenedPos = 0.57;

    public ElapsedTime AccelTimer = new ElapsedTime(), LiftPushingTimer = new ElapsedTime();

    public double TargetX = 0, TargetY = 0, TargetAngle = 90,
            ExtenderDownPos = 0, TargetExtenderPos = 0, LiftDownPos = 0, TargetLiftPos = 0;

    public boolean StopRequested = false, WallPushing = false, NeedToResetExtender = true, NeedToResetLift = true;

    public SampleMecanumDrive Drive;
    public DcMotorEx Extender, LeftLift, RightLift;
    public Servo Swing, LeftPawGear, RightPawGear, LowerClaw,
            MiniExtender, Turret, Elbow, Wrist, UpperClaw;
    public RevTouchSensor ExtenderDownEnd;
    public DigitalChannel LeftLiftDownEnd, RightLiftDownEnd;

    public void Init(HardwareMap hardwareMap, boolean Storage) {
        Drive = new SampleMecanumDrive(hardwareMap);
        Drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(Storage ? AngleStorage : 90)));

        Extender = hardwareMap.get(DcMotorEx.class, "Extender");
        Extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Extender.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        LeftLift = hardwareMap.get(DcMotorEx.class, "LeftLift");
        LeftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LeftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        RightLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        RightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        RightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Swing = hardwareMap.get(Servo.class, "Swing");
        LeftPawGear = hardwareMap.get(Servo.class, "LeftPawGear");
        RightPawGear = hardwareMap.get(Servo.class, "RightPawGear");
        LowerClaw = hardwareMap.get(Servo.class, "LowerClaw");

        MiniExtender = hardwareMap.get(Servo.class, "MiniExtender");
        Turret = hardwareMap.get(Servo.class, "Turret");
        Elbow = hardwareMap.get(Servo.class, "Elbow");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        UpperClaw = hardwareMap.get(Servo.class, "UpperClaw");

        ExtenderDownEnd = hardwareMap.get(RevTouchSensor.class, "ExtenderDownEnd");
        LeftLiftDownEnd = hardwareMap.get(DigitalChannel.class, "LeftLiftDownEnd");
        RightLiftDownEnd = hardwareMap.get(DigitalChannel.class, "RightLiftDownEnd");
    }


    public double ExtenderPos() {
        return Extender.getCurrentPosition() - ExtenderDownPos;
    }

    public double ExtenderPosError() {
        return TargetExtenderPos - ExtenderPos();
    }

    public double ExtenderToPosPower() {
        return ExtenderPosError() * ExtenderKp;
    }

    public void WaitExtender() {
        while (Math.abs(ExtenderPosError()) > 20 && !StopRequested) ;
    }


    public double LiftPosError() {
        return TargetLiftPos - LeftLift.getCurrentPosition() + LiftDownPos;
    }

    public void SetTargetLiftPos(double Pos) {
        LiftPushingTimer.reset();
        TargetLiftPos = Pos;
    }

    public void SetLiftPower(double Power) {
        LeftLift.setPower(Power);
        RightLift.setPower(-Power);
    }

    public void LiftUpdate() {
        double LiftPushingTimerSeconds = LiftPushingTimer.seconds();
        SetLiftPower(NeedToResetLift ? LiftResetPower : Math.max(0, LiftPosError() *
                Math.min(LiftMaxKp, LiftRegularKp + (LiftPushingTimerSeconds > LiftPushingStart ?
                        (LiftPushingTimerSeconds - LiftPushingStart) * LiftPushingAccel : 0))));
        if (NeedToResetLift && (!LeftLiftDownEnd.getState() || !RightLiftDownEnd.getState())) {
            LiftDownPos = LeftLift.getCurrentPosition();
            SetTargetLiftPos(0);
            NeedToResetLift = false;
        }
    }

    public void WaitLift() {
        while (Math.abs(LiftPosError()) > 20 && !StopRequested) ;
    }


    public void SetPaw(double Pos, double Angle) {
        double PawRotation = PawStartRotation + Limit(Angle, 90) * PawAngleMultiply;

        LeftPawGear.setPosition(Pos + PawRotation);
        RightPawGear.setPosition(-Pos + PawRotation);
    }


    public double MinAngleError(double RawAngleError) {
        return RawAngleError + (RawAngleError < -180 ? 360 : RawAngleError > 180 ? -360 : 0);
    }

    public double Limit(double Value, double Module) {
        return Math.max(-Module, Math.min(Module, Value));
    }

    public void Wait(int Milliseconds) {
        ElapsedTime WaitTimer = new ElapsedTime();
        while (WaitTimer.milliseconds() <= Milliseconds && !StopRequested) ;
    }
}
