package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes12833.Renaldo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class Renaldo_DriveTrain {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private ModernRoboticsI2cRangeSensor rangeSensor;
    private Renaldo_VuMarkIdentifier vuMarkIdentifier;
    private BNO055IMU imu;

    private LinearOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();
    private double currentHeading;
    private boolean reverseDriveControl;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    private int flTarget;
    private int frTarget;
    private int blTarget;
    private int brTarget;

    private boolean halfSpeed = false;
    private boolean speedChangeHandled = true;

    //degrees calculations for testbot
    final static double MOTOR_RPM = 160; //ANDYMARK 40 TO 1
    final static double COUNTS_PER_MOTOR_REV = 1120;    // AndyMark
    final static double DRIVE_GEAR_REDUCTION = .62;
    final static double WHEEL_DIAM = 4;
    final static double OUTPUT_RPS = MOTOR_RPM / 60;
    final static double DRIVE_POWER = .18;
    final static double DRIVE_POWER_STRAFE = 1;
    final static double DRIVE_RPS = MOTOR_RPM / 60;
    final static double DRIVE_INCHES_PER_SEC = DRIVE_RPS * WHEEL_DIAM * Math.PI;
    final static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAM * 3.1415);
    final static double TURN_POWER = .8;
    final static double WHEEL_BASE = 15.5;
    final static double TURN_RPS = TURN_POWER * OUTPUT_RPS;
    final static double TURN_DEGRESS_PER_SEC = TURN_RPS * WHEEL_DIAM * 360 / WHEEL_BASE;

    final static double MIN_TO_MOVE = .15;
    final static double RANGE_TOLERANCE = .5;

    final static double TURN_SPEED = 0.35;

    final static double HEADING_THRESHOLD = 1;
    final static double P_TURN_COEFF = 0.09;

    public enum directionToDrive {
        FWRD,
        BACK,
        LEFT,
        RIGHT,
        STOP
    }

    public Renaldo_DriveTrain(LinearOpMode opMode) {
        this.opMode = opMode;

        initializeDriveMotors(opMode);
        setReverseDriveControl(false);

        rangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        if (opMode.getClass() != TeleOp_Renaldo.class) {
            setDriveEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vuMarkIdentifier = new Renaldo_VuMarkIdentifier(opMode);
            vuMarkIdentifier.activateTrackables();
        }

        initializeGyro(opMode);
    }

    private void initializeDriveMotors(LinearOpMode opMode) {
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "flMotor");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frMotor");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "blMotor");
        backRight = opMode.hardwareMap.get(DcMotor.class, "brMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        setDriveEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeGyro(LinearOpMode opMode) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.useExternalCrystal = true;
        parameters.mode = BNO055IMU.SensorMode.IMU;

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void setDriveEncoderMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void driveWithControl() {
        double drive = -opMode.gamepad1.left_stick_y;
        double strafe = opMode.gamepad1.left_stick_x;
        double rotate = opMode.gamepad1.right_stick_x;

        if (reverseDriveControl) {
            drive *= -1;
            strafe *= -1;
//            rotate *= -1;
        }

        frontLeftPower = Math.pow(drive + strafe + rotate, 3);
        frontRightPower = Math.pow(drive - strafe - rotate, 3);
        backLeftPower = Math.pow(drive - strafe + rotate, 3);
        backRightPower = Math.pow(drive + strafe - rotate, 3);

        if (opMode.gamepad1.x) {
            speedChangeHandled = false;
        }

        if (!speedChangeHandled && !opMode.gamepad1.x) {
            halfSpeed = !halfSpeed;
            speedChangeHandled = true;
        }


        if (halfSpeed) {
            frontLeftPower = frontLeftPower / 4;
            frontRightPower = frontRightPower / 4;
            backLeftPower = backLeftPower / 4;
            backRightPower = backRightPower / 4;
        }

        addMinimumPower();   // make sure there is enough power to move the robot
        normalize();   // get all drive powers within -1 to 1 range

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);   // start moving

    }

    public void driveToRange(double target, directionToDrive direction) {
        boolean reachedTarget = false;
        while (!reachedTarget & opMode.opModeIsActive()) {
            double current = getCurrentDistance();
            opMode.telemetry.addData("Driving", direction);
            opMode.telemetry.addData("Range - Target", target);
            opMode.telemetry.addData("      - Current", current);

            if (Math.abs(current - target) <= RANGE_TOLERANCE) {
                reachedTarget = true;
                stopRobot();
            } else {
                if (current > target) {
                    opMode.telemetry.addData("Move", direction);
                    driveDirection(direction);
                } else {
                    opMode.telemetry.addData("Switch to", oppositeDirection(direction));  // too far - switch direction
                    driveDirection(oppositeDirection(direction));
                }
            }
            opMode.telemetry.update();
        }
    }

    public double getCurrentDistance() {
        return rangeSensor.getDistance(DistanceUnit.INCH);
    }

    public RelicRecoveryVuMark getVuMark() {
        return vuMarkIdentifier.getVumark();
    }

    private void driveDirection(directionToDrive direction) {
        switch (direction) {
            case FWRD:
                driveForward();
                break;
            case BACK:
                driveBackward();
                break;
            case LEFT:
                strafeLeft();
                break;
            case RIGHT:
                strafeRight();
                break;
            default:
                stopRobot();
        }
    }

    private directionToDrive oppositeDirection(directionToDrive direction) {
        switch (direction) {
            case FWRD:
                return directionToDrive.BACK;
            case BACK:
                return directionToDrive.FWRD;
            case LEFT:
                return directionToDrive.RIGHT;
            case RIGHT:
                return directionToDrive.LEFT;
            default:
                return directionToDrive.STOP;
        }
    }

    public void setMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void driveForward() {
        setMotorPower(DRIVE_POWER, DRIVE_POWER, DRIVE_POWER, DRIVE_POWER);
    }

    public void driveBackward() {
        setMotorPower(-DRIVE_POWER, -DRIVE_POWER, -DRIVE_POWER, -DRIVE_POWER);
    }

    public void strafeLeft() {
        setMotorPower(-DRIVE_POWER, DRIVE_POWER, DRIVE_POWER, -DRIVE_POWER);
    }

    public void strafeRight() {
        setMotorPower(DRIVE_POWER, -DRIVE_POWER, -DRIVE_POWER, DRIVE_POWER);
    }

    public void stopRobot() {
        setMotorPower(0, 0, 0, 0);
    }

    public void driveForwardTime(double sec, double power) {

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < sec)) {
            opMode.telemetry.addData("drive time", "forward: %2.2f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stopRobot();
    }

    public void brakeOn() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveBackwardTime(double sec, double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < sec)) {
            opMode.telemetry.addData("drive time", "backward: %2.2f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stopRobot();
    }

    public void turnRightTime(double sec, double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < sec)) {
            opMode.telemetry.addData("drive time", "turn right %2.2f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stopRobot();
    }

    public void turnLeftTime(double sec, double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < sec)) {
            opMode.telemetry.addData("drive time", "turn right %2.2f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stopRobot();
    }

    public void turnRightDegree(double degrees) {
        double secondsToPointTurn = degrees / TURN_DEGRESS_PER_SEC;
        turnRightTime(secondsToPointTurn, TURN_POWER);
    }

    public void turnLeftDegree(double degrees) {
        double secondsToPointTurn = degrees / TURN_DEGRESS_PER_SEC;
        turnLeftTime(secondsToPointTurn, TURN_POWER);
    }

    public void driveForwardInches(double inches) {
        double secondsToDrive = inches / DRIVE_INCHES_PER_SEC;
        driveForwardTime(secondsToDrive, DRIVE_POWER);
    }

    public void driveBackwardInches(double inches) {
        double secondsToDrive = inches / DRIVE_INCHES_PER_SEC;
        driveBackwardTime(secondsToDrive, DRIVE_POWER);
    }

    public void strafeRightTime(double sec, double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < sec)) {
            opMode.telemetry.addData("drive time", "strafe right %2.2f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stopRobot();
    }

    public void strafeLeftTime(double sec, double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < sec)) {
            opMode.telemetry.addData("drive time", "strafe right %2.2f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stopRobot();
    }

    public void strafeRightInches(double inches) {
        double secondsToDrive = inches / DRIVE_INCHES_PER_SEC;
        strafeRightTime(secondsToDrive, DRIVE_POWER_STRAFE);
    }

    public void strafeLeftInches(double inches) {
        double secondsToDrive = inches / DRIVE_INCHES_PER_SEC;
        strafeLeftTime(secondsToDrive, DRIVE_POWER_STRAFE);
    }

    public void encoderDrive(double speed, double inches, double timeoutS) {
        int adjustTicks = (int) (inches * COUNTS_PER_INCH);
        adjustDriveTargets(adjustTicks);

        setDriveEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDriveEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(DRIVE_POWER, DRIVE_POWER, DRIVE_POWER, DRIVE_POWER);

        waitForDriveTargets();
        stopRobot();
    }

    private void waitForDriveTargets() {
        while (opMode.opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            opMode.telemetry.addData("Targets", " %7d :%7d : %7d : %7d", flTarget, frTarget, blTarget, brTarget);
            currentDrivePositionTelemetry();

            opMode.telemetry.update();
        }
    }

    public void currentDrivePositionTelemetry() {
        opMode.telemetry.addData("Current", " %7d :%7d : %7d : %7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
    }

    private void adjustDriveTargets(int adjustTicks) {
        flTarget = frontLeft.getCurrentPosition() + adjustTicks;
        frTarget = frontRight.getCurrentPosition() + adjustTicks;
        blTarget = backLeft.getCurrentPosition() + adjustTicks;
        brTarget = backRight.getCurrentPosition() + adjustTicks;

        setDriveTargets(flTarget, frTarget, blTarget, brTarget);
    }

    private void setDriveTargets(int flTarget, int frTarget, int blTarget, int brTarget) {
        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);
    }

    private void normalize() {
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
    }

    private void addMinimumPower() {
        if (frontLeftPower > 0) frontLeftPower += MIN_TO_MOVE;
        else if (frontLeftPower < 0) frontLeftPower -= MIN_TO_MOVE;

        if (frontRightPower > 0) frontRightPower += MIN_TO_MOVE;
        else if (frontRightPower < 0) frontRightPower -= MIN_TO_MOVE;

        if (backLeftPower > 0) backLeftPower += MIN_TO_MOVE;
        else if (backLeftPower < 0) backLeftPower -= MIN_TO_MOVE;

        if (backRightPower > 0) backRightPower += MIN_TO_MOVE;
        else if (backRightPower < 0) backRightPower -= MIN_TO_MOVE;
    }

    public void gyroTurn(double speed, double angle) {
        setDriveEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            opMode.telemetry.update();
        }
    }

    boolean onHeading(double speed, double targetAngle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double frontleftspeed = 0;
        double frontrightspeed = 0;
        double backleftspeed = 0;
        double backrightspeed = 0;

        error = getError(targetAngle);

        if (Math.abs(error) <= HEADING_THRESHOLD) { //error is within acceptable range
            steer = 0.0;
            stopRobot();
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            frontrightspeed = speed * steer;
            frontleftspeed = -frontrightspeed;
            backrightspeed = speed * steer;
            backleftspeed = -frontrightspeed;

            setMotorPower(frontleftspeed, frontrightspeed, backleftspeed, backrightspeed);
        }

        opMode.telemetry.addData("Current angle", getCurrentHeading());
        opMode.telemetry.addData("Target", "%5.2f", targetAngle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed.", "%5.2f:%5.2f:%5.2f:%5.2f", frontleftspeed, frontrightspeed, backleftspeed, backrightspeed);

        return onTarget;
    }

    public void setReverseDriveControl(boolean reverseDriveControl) {
        this.reverseDriveControl = reverseDriveControl;
    }

    double getError(double targetAngle) {
        double robotError;

        currentHeading = getCurrentHeading();
        robotError = targetAngle - currentHeading;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        if (robotError == 180) robotError = 179;
        return robotError;
    }

    public double getCurrentHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}