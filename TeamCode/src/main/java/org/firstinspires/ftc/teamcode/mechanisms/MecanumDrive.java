package org.firstinspires.ftc.teamcode.mechanisms;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrive {
    private static final double TICKS_PER_REV = 28 * 19.2; // =for NeveRest 19.2:1 motor
    private static final double GEAR_RATIO = 60.0 / 40.0; // Motor : Wheel = 60: 40
    private static final double MAX_MOTOR_RPM = 6600 / 19.2; // for NeveRest 19.2:1 motor
    private static final double MAX_MOTOR_TICKS_PER_SEC = MAX_MOTOR_RPM * TICKS_PER_REV / 60;
    private static final double MIN_MOTOR_TICKS_PER_SEC = 0.1 * MAX_MOTOR_TICKS_PER_SEC;
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double IN_PER_REV = Math.PI * WHEEL_DIAMETER_IN * GEAR_RATIO; // how many inches per motor rev
    private static final double TICKS_PER_INCH = TICKS_PER_REV / IN_PER_REV; // how many motor ticks per inch traveled
    private static final double kP = 12.0; // test out
    private static final double kI = 3.0; // test out
    private static final double kD = 0.0; // test out
    private static final double kF = kP / MAX_MOTOR_TICKS_PER_SEC;
    private static final double kP_HEADING = 0.015; // 0.010 ~ 0.030 typical; raise if it under-corrects
    private static final double kP_TURN = 0.012; // 0.010 ~ 0.020 typical; increase if turn is sluggish
    private static final double TOLERANCE_DEG = 1.5; // how close is “good enough”
    private static final int    SETTLE_LOOPS = 6;    // how many consecutive loops inside tolerance before stopping
    private final DcMotorEx[] motors = new DcMotorEx[4];

    public void init(HardwareMap hardwareMap) {
        motors[0] = hardwareMap.get(DcMotorEx.class, "frontleft");
        motors[1] = hardwareMap.get(DcMotorEx.class, "frontright");
        motors[2] = hardwareMap.get(DcMotorEx.class, "backleft");
        motors[3] = hardwareMap.get(DcMotorEx.class, "backright");

        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[3].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.FORWARD);

        for (int i = 0; i < 4; i++) {
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV * 60;
    }

    private double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    private double angleWrapDeg(double d) {
        while (d > 180) d -= 360;
        while (d <= -180) d += 360;
        return d;
    }

    private void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower,
                           double backRightPower) {
        double maxSpeed = 1.0;
        double[] powers = new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, Math.abs(powers[i]));
        }

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i] / maxSpeed);
        }
    }

    public double[] getWheelRpm() {
        double[] wheelRpm = new double[4];
        for (int i = 0; i < 4; i++) {
            wheelRpm[i] = ticksPerSecToRpm(motors[i].getVelocity() * GEAR_RATIO);
        }
        return wheelRpm;
    }

    public double percentMaxRpm(double percentMaxRpm) {
        return percentMaxRpm * MAX_MOTOR_RPM;
    }

    public void runUsingEncoders() {
        for (int i = 0; i < 4; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean isMotorBusy() {
        boolean isBusy = false;
        for (int i = 0; i < 4; i++) {
            isBusy = isBusy || motors[i].isBusy();
        }
        return isBusy;
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void stopDrive() {
        for (int i = 0; i < 4; i++) {
            motors[i].setVelocity(0);
        }
    }

    public void driveForward(double wheelRpm) {
        for (int i = 0; i < 4; i++) {
            motors[i].setVelocityPIDFCoefficients(kP, kI, kD, kF);
            motors[i].setVelocity(rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        }
    }

    public void driveBackward(double wheelRpm) {
        for (int i = 0; i < 4; i++) {
            motors[i].setVelocityPIDFCoefficients(kP, kI, kD, kF);
            motors[i].setVelocity(-rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        }
    }

    public void rotateRight(double wheelRpm) {
        for (int i = 0; i < 2; i++) {
            motors[2 * i].setVelocityPIDFCoefficients(kP, kI, kD, kF);
            motors[2 * i].setVelocity(rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
            motors[2 * i + 1].setVelocityPIDFCoefficients(kP, kI, kD, kF);
            motors[2 * i + 1].setVelocity(-rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        }
    }

    public void rotateLeft(double wheelRpm) {
        for (int i = 0; i < 2; i++) {
            motors[2 * i].setVelocityPIDFCoefficients(kP, kI, kD, kF);
            motors[2 * i].setVelocity(-rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
            motors[2 * i + 1].setVelocityPIDFCoefficients(kP, kI, kD, kF);
            motors[2 * i + 1].setVelocity(rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        }
    }

    public void strafeRight(double wheelRpm) {
        for (int i = 0; i < 4; i++) {
            motors[i].setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
        motors[0].setVelocity(rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        motors[1].setVelocity(-rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        motors[2].setVelocity(-rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        motors[3].setVelocity(rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
    }

    public void strafeLeft(double wheelRpm) {
        for (int i = 0; i < 4; i++) {
            motors[i].setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
        motors[0].setVelocity(-rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        motors[1].setVelocity(rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        motors[2].setVelocity(rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
        motors[3].setVelocity(-rpmToTicksPerSec(wheelRpm / GEAR_RATIO));
    }

    public double forwardRunToTargetPosition(double inches, double baseRPM) {
        int targetTicks = (int) Math.round(inches * TICKS_PER_INCH);
        for (int i = 0; i < 4; i++) {
            motors[i].setTargetPosition(motors[i].getCurrentPosition() + targetTicks);
            motors[i].setTargetPositionTolerance(10);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        return rpmToTicksPerSec(baseRPM);
    }

    public void forwardAdjustYawError(double yawError, double baseVelocityTps) {
        double err = angleWrapDeg(yawError);
        double turnBias = kP_HEADING * err * Math.abs(baseVelocityTps);
        double leftVel = baseVelocityTps - turnBias;
        double rightVel = baseVelocityTps + turnBias;
        leftVel = Range.clip(leftVel, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        rightVel = Range.clip(rightVel, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        for (int i = 0; i < 2; i++) {
            motors[2 * i].setVelocity(leftVel);
            motors[2 * i + 1].setVelocity(rightVel);
        }
    }

    public double strafeRunToTargetPosition(double inches, double baseRPM, boolean right) {
        int targetTicks = (int) Math.round(Math.abs(inches) * TICKS_PER_INCH);
        if (!right) targetTicks = - targetTicks; // left as negative in target pattern
        motors[0].setTargetPosition(motors[0].getCurrentPosition() + targetTicks);
        motors[1].setTargetPosition(motors[1].getCurrentPosition() - targetTicks);
        motors[2].setTargetPosition(motors[2].getCurrentPosition() - targetTicks);
        motors[3].setTargetPosition(motors[3].getCurrentPosition() + targetTicks);
        for (int i = 0; i < 4; i++) {
            motors[i].setTargetPositionTolerance(10);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        return rpmToTicksPerSec(baseRPM);
    }

    public void strafeAdjustYawError(double yawError, double baseVelocityTps) {
        double err = angleWrapDeg(yawError);
        double turnBias = kP_HEADING * err * Math.abs(baseVelocityTps);
        double[] adjustedVelocity = new double[4];
        adjustedVelocity[0] = Range.clip(baseVelocityTps - turnBias, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        adjustedVelocity[1] = Range.clip(- baseVelocityTps - turnBias, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        adjustedVelocity[2] = Range.clip(- baseVelocityTps + turnBias, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        adjustedVelocity[3] = Range.clip(baseVelocityTps + turnBias, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);

        for (int i = 0; i < 4; i++) {
            motors[i].setVelocity(adjustedVelocity[i]);
        }
    }

    public boolean turnAdjustYawErr(double yawErr, int settled) {
        double err = angleWrapDeg(yawErr);
        // stop if within tolerance for several consecutive loops
        if (Math.abs(err) <= TOLERANCE_DEG) {
            if (++settled >= SETTLE_LOOPS) return true;
        } else {
            settled = 0;
        }
        // Proportional velocity command (scale by MAX_MOTOR_TICKS_PER_SEC)
        double turnVel = kP_TURN * Math.abs(err) * MAX_MOTOR_TICKS_PER_SEC;

        // ensure we overcome stiction but don’t exceed limits
        turnVel = Range.clip(turnVel, MIN_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);

        // Positive error => CCW: left backward, right forward
        double leftVel  = (err > 0) ? -turnVel :  turnVel;
        double rightVel = (err > 0) ?  turnVel : -turnVel;
        for (int i = 0; i < 2; i++) {
            motors[2 * i].setVelocity(leftVel);
            motors[2 * i + 1].setVelocity(rightVel);
        }
        return false;
    }

    public double setHeadingDeg(double currentYaw, double deltaYaw) {
        return angleWrapDeg(currentYaw + deltaYaw);
    }

}
