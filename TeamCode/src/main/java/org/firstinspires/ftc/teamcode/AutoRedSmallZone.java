package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PushBar;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherControl;
import org.firstinspires.ftc.teamcode.mechanisms.YawControl;

@Autonomous
public class AutoRedSmallZone extends LinearOpMode{

    private static final int    SETTLE_LOOPS = 6;    // how many consecutive loops inside tolerance before stopping
    MecanumDrive drive = new MecanumDrive();
    PushBar bar = new PushBar();
    IntakeControl intake = new IntakeControl();
    LauncherControl launch = new LauncherControl();
    YawControl robotYaw = new YawControl();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        bar.init(hardwareMap);
        intake.init(hardwareMap);
        launch.init(hardwareMap);
        robotYaw.init(hardwareMap);
        robotYaw.resetYaw();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        if(isStopRequested()) return;

        driveForwardInchesVel(48, drive.percentMaxRpm(0.5), 0.0, 5.0);
        sleep(250);
        driveStrafeInchesVel(24, drive.percentMaxRpm(0.4), 0.0, 6, true);
        sleep(250);
        turnByDeg(45, 2.0);
        sleep(200);
        shooting(6500.0);
        turnToHeadingDeg(0.0, 2.5);
        sleep(200);
        driveForwardInchesVel(-12, drive.percentMaxRpm(0.5), 0.0, 3.5);
        drive.stopDrive();

    }

    private void driveForwardInchesVel(double inches, double baseRPM, double targetDeg, double timeoutSec) {
        double baseVelTps = drive.forwardRunToTargetPosition(inches, baseRPM);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec && drive.isMotorBusy()) {
            double yawErr = targetDeg - robotYaw.getYaw();
            drive.forwardAdjustYawError(yawErr, baseVelTps);
            telemetry.addData("Mode", "Straight");
            telemetry.addData("Target - Yaw Error", "%.1f - %.1f", targetDeg, yawErr);
            telemetry.update();
        }
        drive.stopDrive();
        drive.runUsingEncoders();
    }

    private void driveStrafeInchesVel(double inches, double baseRPM, double targetDeg, double timeoutSec, boolean right) {
        double baseVelTps = drive.strafeRunToTargetPosition(inches, baseRPM, right);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec && drive.isMotorBusy()) {
            double yawErr = targetDeg - robotYaw.getYaw();
            drive.strafeAdjustYawError(yawErr, baseVelTps);
            telemetry.addData("Mode", "Strafe" + (right ? "Right" : "Left"));
            telemetry.addData("Target - Yaw Error", "%.1f - %.1f", targetDeg, yawErr);
            telemetry.update();
        }
        drive.stopDrive();
        drive.runUsingEncoders();
    }

    private void turnToHeadingDeg(double targetDeg, double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        int settled = 0;
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double yawErr = targetDeg - robotYaw.getYaw();
            if(drive.turnAdjustYawErr(yawErr) == 1 && ++settled >= SETTLE_LOOPS) break;
            telemetry.addData("Target - Yaw Error", "%.1f - %.1f", targetDeg, yawErr);
            telemetry.update();
        }
        drive.stopDrive();
        drive.runUsingEncoders();
    }

    private void turnByDeg(double deltaDeg, double timeoutSec) {
        double start = robotYaw.getYaw();
        double target = drive.setHeadingDeg(start, deltaDeg);
        turnToHeadingDeg(target, timeoutSec);
    }

    private void shooting(double wheelTargetRpm) {
        launch.startLaunch(wheelTargetRpm);
        ElapsedTime spin = new ElapsedTime();
        spin.reset();
        while (opModeIsActive() && spin.seconds() < 2.0) sleep(10);
        for(int i = 0; i < 3 && opModeIsActive(); i++) {
            sleep(250);
            bar.pushBall(0.55, 0.2);
            sleep(500);
            bar.release(0.65, 1.0);
        }
        launch.stopLaunch();
    }

}
