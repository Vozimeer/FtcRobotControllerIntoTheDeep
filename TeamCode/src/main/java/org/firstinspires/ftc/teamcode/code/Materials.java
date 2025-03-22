package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Config
public class Materials {
    // 192.168.43.1:8080/dash
    public static double AngleStorage = 90, ExtenderCorrectionZone = 200,
            ExtenderKp = 0.02, ExtenderResetPower = -0.8, ExtenderResetMilliseconds = 100, ExtenderMaxPos = 600,

    LiftMinKp = 0.006, LiftMaxKp = 0.04, LiftPushingStartSeconds = 0.6, LiftPushingAccel = 0.04,
            LiftResetDownPower = -0.2, LiftResetDownMilliseconds = 100, LiftResetUpPower = 0.08, LiftResetMilliseconds = 700,
            LiftThrowPos = 250, LiftClippingPos = 500, LiftBasketPos = 1200,

    SwingInsidePos = 0.04, SwingTransferPos = 0.26, SwingCheckPos = 0.3, SwingPreparePos = 0.47, SwingBottomPos = 0.58,
            PawStartPos = 0, PawFoldPos = 0.05, PawThrowPos = 0.16, PawTransferPos = 0.38, PawStartRotation = 0.532, PawAngleMultiply = 0.0017,
            LowerClawHardPos = 0.82, LowerClawSoftPos = 0.7694, LowerClawMidPos = 0.71, LowerClawOpenedPos = 0.6,

    MiniExtenderTransferPos = 0.413, MiniExtenderWallPos = 0.47, MiniExtenderClippingPos = 0.68,
            ElbowTransferPos = 0.75, ElbowTransferPreparePos = 0.58, ElbowClippingPos = 0.7,
            ElbowVerticalPos = 0.46, ElbowBasketPos = 0.3, ElbowWallPreparePos = 0.25, ElbowWallPos = 0.2,
            WristTransferPos = 0.7, WristClippingPos = 0.6, WristStraightPos = 0.52, WristWallPos = 0.329,
            UpperClawOpenedPos = 0.79, UpperClawClosedPos = 1;

    public SampleDetectionPipeline SDP;
    public ElapsedTime ExtenderResetTimer = new ElapsedTime(),
            LiftResetTimer = new ElapsedTime(), LiftPushingTimer = new ElapsedTime();

    public double ExtenderRawPos = 0, ExtenderDownPos = 0, TargetExtenderPos = 0, LiftRawPos = 0, LiftDownPos = 0;
    public boolean StopRequested = false, NeedToResetExtender = true, NeedToResetLift = true;
    public int TargetLiftState = 0;

    public SampleMecanumDrive Drive;
    public DcMotorEx Extender, LeftLift, RightLift;
    public Servo Swing, LeftPawGear, RightPawGear, LowerClaw,
            MiniExtender, Elbow, Wrist, UpperClaw;
    public RevTouchSensor ExtenderDownEnd;
    public DigitalChannel LeftLiftDownEnd, RightLiftDownEnd;
    public OpenCvWebcam Webcam;

    public void Init(HardwareMap hardwareMap, boolean Storage) {
        List<LynxModule> AllHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule Hub : AllHubs) {
            Hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Drive = new SampleMecanumDrive(hardwareMap);
        Drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(Storage ? AngleStorage : 90)));

        Extender = hardwareMap.get(DcMotorEx.class, "Extender");
        Extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Extender.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        LeftLift = hardwareMap.get(DcMotorEx.class, "LeftLift");
        LeftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
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
        Elbow = hardwareMap.get(Servo.class, "Elbow");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        UpperClaw = hardwareMap.get(Servo.class, "UpperClaw");

        ExtenderDownEnd = hardwareMap.get(RevTouchSensor.class, "ExtenderDownEnd");
        LeftLiftDownEnd = hardwareMap.get(DigitalChannel.class, "LeftLiftDownEnd");
        RightLiftDownEnd = hardwareMap.get(DigitalChannel.class, "RightLiftDownEnd");
    }

    public void InitOpenCV(HardwareMap hardwareMap, boolean Red) {
        int CameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), CameraMonitorViewId);
        SDP = new SampleDetectionPipeline(Red);
        Webcam.setPipeline(SDP);
        Webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        FtcDashboard.getInstance().startCameraStream(Webcam, 24);
    }


    public void SetFieldOrientedDrivePower(Vector2D TranslationalPowerVec, double TurnPower, double ExtenderCorrectionMultiply) {
        TranslationalPowerVec.rotateBy(-Drive.getPoseEstimate().getHeading());
        if (ExtenderPos() < ExtenderCorrectionZone) ExtenderCorrectionMultiply = 1;
        double FLS = -TranslationalPowerVec.y * ExtenderCorrectionMultiply + TranslationalPowerVec.x - TurnPower,
                BLS = TranslationalPowerVec.y + TranslationalPowerVec.x - TurnPower,
                BRS = -TranslationalPowerVec.y + TranslationalPowerVec.x + TurnPower,
                FRS = TranslationalPowerVec.y * ExtenderCorrectionMultiply + TranslationalPowerVec.x + TurnPower,
                SMax = Math.max(Math.max(Math.abs(FLS), Math.abs(BLS)), Math.max(Math.abs(BRS), Math.abs(FRS)));
        if (SMax > 1) {
            FLS /= SMax;
            BLS /= SMax;
            BRS /= SMax;
            FRS /= SMax;
        }
        Drive.setMotorPowers(FLS, BLS, BRS, FRS);
        Drive.update();
    }


    public double ExtenderPos() {
        return ExtenderRawPos - ExtenderDownPos;
    }

    public double ExtenderPosError() {
        return TargetExtenderPos - ExtenderPos();
    }

    public double ExtenderToPosPower() {
        return ExtenderPosError() * ExtenderKp;
    }

    public void ExtenderAutoUpdate() {
        ExtenderRawPos = Extender.getCurrentPosition();
        Extender.setPower(NeedToResetExtender ? Materials.ExtenderResetPower : ExtenderToPosPower());
        if (NeedToResetExtender && ExtenderDownEnd.isPressed()) {
            if (ExtenderResetTimer.milliseconds() > Materials.ExtenderResetMilliseconds) {
                ExtenderDownPos = ExtenderRawPos;
                TargetExtenderPos = 0;
                ExtenderResetTimer.reset();
                NeedToResetExtender = false;
            }
        } else ExtenderResetTimer.reset();
    }


    public double LiftPosError() {
        int LocalTargetLiftState = TargetLiftState;
        return (LocalTargetLiftState == 1 ? LiftThrowPos : LocalTargetLiftState == 2 ? LiftClippingPos :
                LocalTargetLiftState == 3 ? LiftBasketPos : 0) - LiftRawPos + LiftDownPos;
    }

    public void SetLiftPower(double Power) {
        LeftLift.setPower(-Power);
        RightLift.setPower(Power);
    }

    public void SetTargetLiftState(int State) {
        LiftPushingTimer.reset();
        TargetLiftState = State;
    }

    public void LiftUpdate() {
        LiftRawPos = LeftLift.getCurrentPosition();
        double LiftResetTimerMilliseconds = LiftResetTimer.milliseconds();
        boolean ResettingUp = LiftResetTimerMilliseconds >= LiftResetDownMilliseconds;
        SetLiftPower(NeedToResetLift ? ResettingUp ? LiftResetUpPower : LiftResetDownPower : Math.max(0, LiftPosError() *
                Math.min(LiftMaxKp, LiftMinKp + (Math.max(0, LiftPushingTimer.seconds() - LiftPushingStartSeconds) * LiftPushingAccel))));
        if ((NeedToResetLift && (!LeftLiftDownEnd.getState() || !RightLiftDownEnd.getState())) || ResettingUp) {
            if (LiftResetTimerMilliseconds >= LiftResetMilliseconds) {
                LiftDownPos = LiftRawPos;
                SetTargetLiftState(0);
                LiftResetTimer.reset();
                NeedToResetLift = false;
            }
        } else LiftResetTimer.reset();
    }


    public void SetPaw(double Pos, double Angle) {
        double PawRotation = PawStartRotation + Limit(Angle, 90) * PawAngleMultiply;

        LeftPawGear.setPosition(-PawStartPos - Pos + PawRotation);
        RightPawGear.setPosition(PawStartPos + Pos + PawRotation);
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
