package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes12833.Archimedes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Archimedes_Drivetrain {

    private LinearOpMode opMode;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private Servo phoneMount;

    private boolean halfSpeed = false;
    private boolean isHandled = false;

    double flMotorPower;
    double frMotorPower;
    double blMotorPower;
    double brMotorPower;
    double max = 1;

    public Archimedes_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        //init motors
        frontLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, "FLMotor");
        frontRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "FRMotor");
        backLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, "BLMotor");
        backRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "BRMotor");
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        phoneMount = opMode.hardwareMap.get(Servo.class, "phoneMount");
        phoneMount.setPosition(0);
    }

    public void driveWithSticks() {
        double drive = -opMode.gamepad1.left_stick_y;
        double turn = opMode.gamepad1.right_stick_x;
        double strafe = opMode.gamepad1.left_stick_x;

        if (Math.abs(drive) + Math.abs(turn) + Math.abs(strafe) > 1) {
            max = Math.abs(drive) + Math.abs(turn) + Math.abs(strafe);
        }

        flMotorPower = (drive + turn + strafe) / max;
        frMotorPower = (drive - turn - strafe) / max;
        blMotorPower = (drive + turn - strafe) / max;
        brMotorPower = (drive - turn + strafe) / max;

        if (opMode.gamepad1.a && !isHandled) {
            halfSpeed = !halfSpeed;
            isHandled = true;
        } else if (!opMode.gamepad1.a) {
            isHandled = false;
        }

        if (halfSpeed) {
            flMotorPower = flMotorPower / 2;
            frMotorPower = frMotorPower / 2;
            blMotorPower = blMotorPower / 2;
            brMotorPower = brMotorPower / 2;
        }

        frontLeftMotor.setPower(flMotorPower);
        frontRightMotor.setPower(frMotorPower);
        backLeftMotor.setPower(blMotorPower);
        backRightMotor.setPower(brMotorPower);
        max = 1;
    }
}