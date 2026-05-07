package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes12833.misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TrapBot", group="MM99")
//@Disabled
public class TeleOp_TrapBot extends LinearOpMode {
    private DcMotorEx leftMotor = null;
    private DcMotorEx rightMotor = null;
    private Servo gateRight = null;
    private Servo gateLeft = null;
    private Servo lifter = null;

    private Gamepad previousGamepad = new Gamepad();
    private Gamepad currentGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        gateRight = hardwareMap.get(Servo.class, "gateRight");
        lifter = hardwareMap.get(Servo.class, "lifter");
        gateLeft = hardwareMap.get(Servo.class, "gateLeft");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gateLeft.setPosition(0);
        lifter.setPosition(0);
        gateRight.setPosition(0);

        telemetry.addData("Init status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            try {previousGamepad.copy(currentGamepad);}
            catch (RobotCoreException e) {e.printStackTrace();}

            try {currentGamepad.copy(gamepad1);}
            catch (RobotCoreException e) {e.printStackTrace();}


            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                leftMotor.setPower(-gamepad1.left_stick_y);
                rightMotor.setPower(-gamepad1.left_stick_y);
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                leftMotor.setPower(-gamepad1.right_stick_x);
                rightMotor.setPower(gamepad1.right_stick_x);
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
            if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                gateLeft.setPosition(0); // open
                gateRight.setPosition(0); //open
                telemetry.addData("Gate Status", "open");
            }
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
                gateLeft.setPosition(1); // closed
                gateRight.setPosition(1); //closed
                telemetry.addData("Gate Status", "closed");
            }
            if (currentGamepad.left_trigger > 0.1 && previousGamepad.left_trigger < 0.1) {
                lifter.setPosition(0);
                telemetry.addData("Lifter Status", "open");
            }
            if (currentGamepad.right_trigger > 0.1 && previousGamepad.right_trigger < 0.1) {
                lifter.setPosition(1);
                telemetry.addData("Lifter Status", "closed");
            }
        telemetry.update();
        }
    }
}
