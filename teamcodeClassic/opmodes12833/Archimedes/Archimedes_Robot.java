package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Archimedes_Robot {
    private LinearOpMode opMode;
    public Archimedes_Drivetrain drivetrain;
    public Archimedes_Collector collector;
    public Archimedes_Shooter shooter;
    public Archimedes_WobbleScorer wobbleScorer;

    boolean usePID = true;
    boolean isHandled = false;

    private ElapsedTime runtime = new ElapsedTime();

    public Archimedes_Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void autoLaunch() {
        if(opMode.gamepad2.right_stick_button && !isHandled){
            isHandled = true;
            usePID = !usePID;
        }
        if(!opMode.gamepad2.right_stick_button && isHandled){
            isHandled = false;
        }
        if(usePID){
            opMode.telemetry.addLine("Using PID Shoot");
            if (opMode.gamepad2.right_bumper && collector.ringsDetected() > 0) {
                launchOneRing(.001);
            } else if (opMode.gamepad2.right_trigger > .01 && collector.ringsDetected() > 0) {
                launchMultiRings(.001);
            } else if (opMode.gamepad2.y) {
                cancelAutoLaunch();
                shooter.launcherTelemetry();
                opMode.telemetry.addData("Stopping Transport", " ");
            } else {
                shooter.manualShoot();
            }
        }else{
            if (opMode.gamepad2.right_bumper && collector.ringsDetected() > 0) {
                launchOneRing();
            } else if (opMode.gamepad2.right_trigger > .01 && collector.ringsDetected() > 0) {
                launchMultiRings();
            } else if (opMode.gamepad2.y) {
                cancelAutoLaunch();
                shooter.launcherTelemetry();
                opMode.telemetry.addData("Stopping Transport", " ");
            } else {
                shooter.manualShoot();
            }
            opMode.telemetry.addLine("Using Normal Shoot");
        }
    }

    public void launchOneRing() {
        shooter.runLauncherAtSpeed();

        runtime.reset();
        while (opMode.opModeIsActive() && shooter.checkVelocity() < shooter.getMinVelocityForGate() && runtime.seconds() < 1.0) { //timeout was originally 3 seconds
            drivetrain.driveWithSticks();
            shooter.launcherTelemetry();
            opMode.telemetry.update();

            if (opMode.gamepad2.y) {
                cancelAutoLaunch();
                return;
            }
        }

        collector.runStageWheel();

        while(collector.ringsDetected() > 0){
            drivetrain.driveWithSticks();
            shooter.launcherTelemetry();
            opMode.telemetry.update();
        }

        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 1){
            drivetrain.driveWithSticks();
        }

        collector.stopStageWheel();
    }

    public void launchOneRing(double pCoeff) {
        runtime.reset();
        while (opMode.opModeIsActive() && shooter.checkVelocity() < shooter.getMinVelocityForGate()) {
            shooter.runLauncher(shooter.PLauncher(shooter.getMinVelocityForGate(), pCoeff));
            drivetrain.driveWithSticks();
            shooter.launcherTelemetry();
            opMode.telemetry.update();

            if (opMode.gamepad2.y) {
                cancelAutoLaunch();
                return;
            }
        }

        collector.runStageWheel();

        while(collector.ringsDetected() > 0){
            drivetrain.driveWithSticks();
            shooter.launcherTelemetry();
            opMode.telemetry.update();
        }

        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 1){
            drivetrain.driveWithSticks();
        }

        collector.stopStageWheel();
    }

    public void launchMultiRings() {
        for (int i = 0; i < 3; i++) {
            launchOneRing();
            if(i == 2) return;

            collector.setTransportPowers(-1, 1, -1);

            runtime.reset();
            while (opMode.opModeIsActive() && collector.ringsDetected() < 1 && runtime.seconds() < 4.0) {
                drivetrain.driveWithSticks();
                shooter.launcherTelemetry();
                opMode.telemetry.addData("Trying to load a ring", runtime.seconds());
                opMode.telemetry.update();

                if (opMode.gamepad2.y) {
                    cancelAutoLaunch();
                    return;
                }
            }
            collector.setTransportPowers(0, 0, 0);
        }
    }

    public void launchMultiRings(double pCoeff) {
        for (int i = 0; i < 3; i++) {
            launchOneRing(pCoeff);
            if(i == 2) return;

            collector.setTransportPowers(-1, 1, -1);

            runtime.reset();
            while (opMode.opModeIsActive() && collector.ringsDetected() < 1 && runtime.seconds() < 4.0) {
                drivetrain.driveWithSticks();
                shooter.launcherTelemetry();
                opMode.telemetry.addData("Trying to load a ring", runtime.seconds());
                opMode.telemetry.update();

                if (opMode.gamepad2.y) {
                    cancelAutoLaunch();
                    return;
                }
            }
            collector.setTransportPowers(0, 0, 0);
        }
    }

    private void cancelAutoLaunch() {
        shooter.stopLauncher();
        collector.setTransportPowers(0, 0, 0);
    }

    public void init() {
        drivetrain = new Archimedes_Drivetrain(opMode);
        collector = new Archimedes_Collector(opMode);
        shooter = new Archimedes_Shooter(opMode);
        wobbleScorer = new Archimedes_WobbleScorer(opMode);
    }
}