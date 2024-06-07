package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Hammerhead", group="Hammerhead")
public class Hammerhead extends LinearOpMode {

    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftBack   = null;
    private DcMotor rightBack  = null;

    private Servo arm = null;
    private Servo grabber = null;

    private final double NORMAL_SPEED = .5;
    private final double SLOW_SPEED = .17;

    private boolean slowMode = false;
    private double driveSpeedFactor = NORMAL_SPEED;

    private Gamepad gamepadCurrent = new Gamepad();
    private Gamepad gamepadPrior = new Gamepad();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;

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

            left = driveSpeedFactor * -gamepadCurrent.left_stick_y;
            right = driveSpeedFactor * -gamepadCurrent.right_stick_y;

            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            leftFront.setPower(left);
            leftBack.setPower(left);
            rightFront.setPower(right);
            rightBack.setPower(right);

            if(gamepadCurrent.left_bumper) grabber.setPosition(0);
            if(gamepadCurrent.right_bumper) grabber.setPosition(1);
            if(gamepadCurrent.left_trigger > 0) arm.setPosition(1);
            if(gamepadCurrent.right_trigger > 0) arm.setPosition(0);

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

        arm = hardwareMap.get(Servo.class, "arm");
        grabber = hardwareMap.get(Servo.class, "grabber");

        arm.setPosition(1);
        grabber.setPosition(0);
    }
}
