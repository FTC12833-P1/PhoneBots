package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mecanum Drive", group="mecanums")
public class MecanumDrive extends LinearOpMode {

    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftBack   = null;
    private DcMotor rightBack  = null;

    private final double NORMAL_SPEED = 1;
    private final double SLOW_SPEED = .4;

    private boolean slowMode = false;
    private double driveSpeedFactor = NORMAL_SPEED;

    private Gamepad gamepadCurrent = new Gamepad();
    private Gamepad gamepadPrior = new Gamepad();

    @Override
    public void runOpMode() {
        double flPower = 0;
        double frPower = 0;
        double blPower = 0;
        double brPower = 0;

        initHardware();

        telemetry.addLine("Press Play to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            try{
                gamepadPrior.copy(gamepadCurrent);
                gamepadCurrent.copy(gamepad1);
            } catch(RobotCoreException e){}

            if(gamepadCurrent.a && !gamepadPrior.a){
                slowMode = !slowMode;
                driveSpeedFactor = (slowMode? SLOW_SPEED : NORMAL_SPEED);
            }

            double drive = -gamepadCurrent.left_stick_y;
            double turn = gamepadCurrent.right_stick_x;
            double strafe = gamepadCurrent.left_stick_x;

            flPower = driveSpeedFactor * (drive + turn + strafe);
            frPower = driveSpeedFactor * (drive - turn - strafe);
            blPower = driveSpeedFactor * (drive + turn - strafe);
            brPower = driveSpeedFactor * (drive - turn + strafe);

            double max = Math.max(Math.abs(flPower), Math.abs(frPower));
            if (max > 1.0)
            {
                flPower /= max;
                frPower /= max;
                blPower /= max;
                brPower /= max;
            }

            leftFront.setPower(flPower);
            leftBack.setPower(blPower);
            rightFront.setPower(frPower);
            rightBack.setPower(brPower);

            telemetry.update();
        }
    }

    public void initHardware(){
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }
}
