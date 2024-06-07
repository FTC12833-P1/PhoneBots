package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Archimedes_Shooter {

    private LinearOpMode opMode;

    // Base "guess" was .80 power = 2100 velocity --> +.01 power = +20 velocity
    private static double HIGH_GOAL_SPEED = .87;
    private static double HIGH_GOAL_MIN_VELOCITY = 2240;
    private static double POWER_SHOT_SPEED = .8;
    private static double POWER_SHOT_MIN_VELOCITY = 2100;
    private static double DEFAULT_SPEED = HIGH_GOAL_SPEED;
    private static double DEFAULT_MIN_VELOCITY = HIGH_GOAL_MIN_VELOCITY;

    private DcMotorEx flyWheel;

    private double launchSpeed = DEFAULT_SPEED;
    private double minVelocityForGate = DEFAULT_MIN_VELOCITY;

    public Archimedes_Shooter(LinearOpMode opMode) {
        this.opMode = opMode;

        flyWheel = opMode.hardwareMap.get(DcMotorEx.class, "flywheel");
        flyWheel.setDirection(DcMotor.Direction.FORWARD);
    }

    public double PLauncher(double targetVelocity, double pCoeff){
        double currentVelocity = flyWheel.getVelocity();
        double currentError = targetVelocity - currentVelocity;
        return (currentError * pCoeff) + launchSpeed;
    }

    public void runLauncher(double power){
        flyWheel.setPower(power);
    }

    public void manualShoot() {

        if (opMode.gamepad2.dpad_down) {  // Power Shot
            launchSpeed = POWER_SHOT_SPEED;
            minVelocityForGate = POWER_SHOT_MIN_VELOCITY;
        } else if (opMode.gamepad2.dpad_up) {  // High Goal
            launchSpeed = HIGH_GOAL_SPEED;
            minVelocityForGate = HIGH_GOAL_MIN_VELOCITY;
        }
        stopLauncher();
    }

    public void stopLauncher() {
        flyWheel.setPower(0);
    }

    public void runLauncherAtSpeed() {
        flyWheel.setPower(launchSpeed);
    }

    public double checkVelocity() {
        return flyWheel.getVelocity();
    }

    public void launcherTelemetry(){
        opMode.telemetry.addData("Launch set speed", "%3.2f", launchSpeed);
        opMode.telemetry.addData("Current Velocity", "%5.1f", checkVelocity());
        opMode.telemetry.addData("Target Velocity for Gate", "%5.1f", minVelocityForGate);
    }

    public double getMinVelocityForGate() {
        return minVelocityForGate;
    }
}
