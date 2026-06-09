package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes12833.misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ScissorLift", group = "Linear Opmode")


public class TeleOp_ScissorLift extends LinearOpMode {
    private DcMotorEx flMotor = null;
    private DcMotorEx frMotor = null;
    private DcMotorEx blMotor = null;
    private DcMotorEx brMotor = null;
    private DcMotorEx liftMotor = null;

    double drivePower = -gamepad1.left_stick_y;
    double rotatePower = gamepad1.right_stick_x;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initHardware();
        waitForStart();

        while (opModeIsActive()){

        }
    }

    public void caluclateAndSetDrivePowers() {
        //calculate drive powers

        //set drive powers
        flMotor.setPower(1); //TODO change to be correct, just added 1 so that there was no error
    }

    public void initHardware() {
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        //TODO figure out which motors need to be reversed
    }
}
