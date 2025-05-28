package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes12833.misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="chalkBot", group="mm")
public class TeleOp_ChalkBot extends LinearOpMode {

    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private Servo tail;

    double leftpower;
    double rightPower;
    final double MAX_POWER = 0.4;
    final double MAX_SLOW_POWER = 0.3;
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad1 = new Gamepad();

    private ElapsedTime wagTime = new ElapsedTime();

    private boolean needToWagToZero = false;

    private boolean slow = false;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "initializing please wait");
        telemetry.update();
        initHardware();
        telemetry.addData("Status", "initialized");
        telemetry.update();

        waitForStart();

        //wagTime.startTime();

        while (opModeIsActive()){
            try {
                previousGamepad1.copy(currentGamepad1);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
            try {
                currentGamepad1.copy(gamepad1);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }

            if (currentGamepad1.a && !previousGamepad1.a && !gamepad1.start){
                slow = !slow;
            }

            if (slow) {
                leftpower = Range.clip(gamepad1.left_stick_y - gamepad1.right_stick_x, -MAX_SLOW_POWER, MAX_SLOW_POWER);
                rightPower = Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -MAX_SLOW_POWER, MAX_SLOW_POWER);
            } else {
                leftpower = Range.clip(gamepad1.left_stick_y - gamepad1.right_stick_x, -MAX_POWER, MAX_POWER);
                rightPower = Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -MAX_POWER, MAX_POWER);
            }

            leftDrive.setPower(leftpower);
            rightDrive.setPower(rightPower);

            if (wagTime.milliseconds() >= 500) {
                needToWagToZero = !needToWagToZero;
                wagTime.reset();
            }

            if (needToWagToZero) {
                tail.setPosition(0);
            } else {
                tail.setPosition(1);
            }
        }
    }
    public void initHardware(){
        leftDrive = hardwareMap.get(DcMotorEx.class, "left");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right");
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);

        tail = hardwareMap.get(Servo.class, "tail");
        tail.setPosition(0);
    }
}
