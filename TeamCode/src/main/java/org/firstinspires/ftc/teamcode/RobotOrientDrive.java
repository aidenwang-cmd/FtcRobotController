package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PushBar;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherControl;

import java.util.ArrayList;
import java.util.List;

@TeleOp()
public class RobotOrientDrive extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    PushBar bar = new PushBar();
    IntakeControl intake = new IntakeControl();
    LauncherControl launch = new LauncherControl();
    List<Double> barPositions = new ArrayList<>(2);
    private boolean launching = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private double launchTargetRpm = 6500.0;
    private double launchRpmAdjustment = 500.0;
    double launchRpm;


    @Override
    public  void init() {
        drive.init(hardwareMap);
        bar.init(hardwareMap);
        intake.init(hardwareMap);
        launch.init(hardwareMap);
    }

    @Override
    public void start(){
        bar.pushBall(0.65, 1.0);
    }

    @Override
    public void loop() {
        // use gamepad sticks to control driving
        double forward = gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        drive.drive(forward, right, rotate);

        // press right bumper to swing bars, release to reset
        if(gamepad1.right_bumper) {
            bar.pushBall(0.55, 0.2);
            telemetry.addData("after push", bar.getBarPosition());
        }
        else {
            bar.release(0.65, 1.0);
            telemetry.addData("after release", bar.getBarPosition());
        }

        // intake
        // left trigger: take in balls with adjusted speed
        // left bumper: release jammed balls
        if(gamepad1.left_bumper) {
            intake.setIntakePower(-1.0);
        }
        else {
            intake.setIntakePower(gamepad1.left_trigger);
        }

        // shoot
        // target RPM adjustment: dpad up, dpad down
        // start and stop launching motor: a (start), y (stop)

        // shoot: dpad up - nudge up target RPM every press by a fixed amount
        boolean dpadUp = gamepad1.dpad_up;
        if(dpadUp && !lastDpadUp) {
            launchTargetRpm = launch.adjustLaunchRpm(launchTargetRpm,launchRpmAdjustment);
        }
        lastDpadUp = dpadUp;

        // shoot: dpad down - nudge down target RPM every press by a fixed amount
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !lastDpadDown) {
            launchTargetRpm = launch.adjustLaunchRpm(launchTargetRpm,-launchRpmAdjustment);
        }
        lastDpadDown = dpadDown;

        // shoot: a - start launching wheel at target RPM
        if(gamepad1.a && !launching) {
            launchRpm = launch.startLaunch(launchTargetRpm);
            telemetry.addData("Launch starts, target RPM: ", "6000");
            telemetry.addData("Launch actual RPM: ", (int) launchRpm);
            launching = true;
        }

        // shoot: y - stop launching wheel
        if(gamepad1.y && launching) {
            launchRpm = launch.stopLaunch();
            telemetry.addData("Launch stops, target RPM", "0");
            telemetry.addData("Launch RPM: ", launchRpm);
            launching = false;
        }
        telemetry.update();
    }
}