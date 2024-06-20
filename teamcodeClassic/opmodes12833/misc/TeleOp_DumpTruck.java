package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes12833.misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="dumpTruck", group="mm")
public class TeleOp_DumpTruck extends LinearOpMode {

    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;

    private Servo dumper;

    double leftpower;
    double rightPower;
    private ElapsedTime dumpTime = new ElapsedTime();

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "initializing please wait");
        telemetry.update();
        initHardware();
        telemetry.addData("Status", "initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()){
            leftpower = Range.clip(-gamepad1.left_stick_y + gamepad1.right_stick_x, -1, 1);
            rightPower = Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1 );

            leftDrive.setPower(leftpower);
            rightDrive.setPower(rightPower);

            if(gamepad1.x){
                dumper.setPosition(.56);
            }
            if(gamepad1.y){
                dumper.setPosition(0);
            }
        }
    }
    public void initHardware(){
        leftDrive = hardwareMap.get(DcMotorEx.class, "left");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right");

        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);

        dumper = hardwareMap.get(Servo.class, "dumper");
        dumper.setPosition(0);
    }
}
