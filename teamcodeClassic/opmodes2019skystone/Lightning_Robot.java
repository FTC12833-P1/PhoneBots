package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Lightning_Robot {
    private LinearOpMode opMode;
    public Lightning_Drivetrain drivetrain;
    public Lightning_Collector collector;
    public Lightning_Arm arm;

    private static boolean haveBlock = false;

    public Lightning_Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new Lightning_Drivetrain(opMode);
        collector = new Lightning_Collector(opMode);
        arm = new Lightning_Arm(opMode);
    }

    public static void setHaveBlock(boolean haveBlock) {
        Lightning_Robot.haveBlock = haveBlock;
    }

    public static boolean haveBlock() {
        return haveBlock;
    }

    public void teleOpAutoCollect() {
        if (opMode.gamepad1.x) {
            opMode.resetStartTime();
            arm.autoArm(-1000);
            collector.powerFlywheels(-1);
            while (collector.getCollectorDistance() > 8) {
                drivetrain.driveWithSticks();
            }
            consistentCollect();
            collector.powerFlywheels(0);
        }
    }

    public boolean consistentCollect() {
        boolean haveBlock = false;
        for (int i = 0; i < 2; i++) {
            if (collector.getCollectorDistance() < 2.6) {
                collector.autoAlignStone();
                haveBlock = true;
            } else {
                collector.powerFlywheels(0);
                opMode.resetStartTime();
                while (opMode.opModeIsActive() && opMode.getRuntime() < 1) {
                }
                collector.powerFlywheels(1);
                opMode.resetStartTime();
                while (opMode.opModeIsActive() && opMode.getRuntime() < .25) {
                }
                collector.powerFlywheels(-1);
                opMode.resetStartTime();
                while (opMode.opModeIsActive() && opMode.getRuntime() < .5) {
                }
            }
        }
        return haveBlock;
    }
}
