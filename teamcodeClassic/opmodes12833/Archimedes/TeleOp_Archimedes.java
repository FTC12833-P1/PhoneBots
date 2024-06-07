package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Archimedes", group = "MM2020-2021")
public class TeleOp_Archimedes extends LinearOpMode {

    private Archimedes_Robot robot = new Archimedes_Robot(this);

    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
                robot.autoLaunch();
                robot.drivetrain.driveWithSticks();
                robot.wobbleScorer.controlWobble();
                robot.collector.controlTransport();
            telemetry.update();
        }
    }
}
