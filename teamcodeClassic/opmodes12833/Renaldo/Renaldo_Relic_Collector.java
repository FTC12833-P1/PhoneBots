package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Renaldo_Relic_Collector {

    private DcMotor arm = null;
    private Servo wrist = null;
    private Servo grabber = null;
    private DigitalChannel touchRelicArm = null;

    private double currentGrabberPos;

    private double WRIST_START_POSITION = 0.22;
    private double WRIST_MID_POSITION = .2;
    private double WRIST_LIFT_POSITION = .13;

    private double GRABBER_START_POS = .65;
    private double GRABBER_ADJUST_AMOUNT = .01;
    public double GRABBER_OPEN = 1;
    private double GRABBER_CLOSE = .39;
    public double MAX = 7315;
    public double MID = 5600;

    private LinearOpMode opMode;

    public Renaldo_Relic_Collector(LinearOpMode opMode) {
        this.opMode = opMode;

        arm = opMode.hardwareMap.get(DcMotor.class, "relic_arm");
        wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        grabber = opMode.hardwareMap.get(Servo.class, "grabber");
        touchRelicArm = opMode.hardwareMap.get(DigitalChannel.class, "touch_relic_arm");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist.setPosition(WRIST_START_POSITION);
        grabber.setPosition(GRABBER_START_POS);

        currentGrabberPos = GRABBER_START_POS;
    }

    public void setArmPower(double power) {
        arm.setPower(power);
        opMode.telemetry.addData("Arm Power ", power);
    }
    public void grabberOpen() {
        setGrabberPosition(GRABBER_OPEN);
    }
    public void grabberClose() {
        setGrabberPosition(GRABBER_CLOSE);
    }
    public void raiseWrist() {
        setWristPosition(WRIST_LIFT_POSITION);
    }
    public void midWrist() {
        setWristPosition(WRIST_MID_POSITION);
    }
    public void lowerWrist() {
        setWristPosition(WRIST_START_POSITION);
    }
    public void setWristPosition(double inPosition) {
        wrist.setPosition(inPosition);
        opMode.telemetry.addData("Wrist Position ", inPosition);
    }
    public void setGrabberPosition(double inPosition) {
        grabber.setPosition(inPosition);
    }
    public double getCurrentGrabberPos() {
        return currentGrabberPos;
    }
    public double getCurrentArmPos() {
        return arm.getCurrentPosition();
    }
    public boolean relicArmRetracted() {
        if (touchRelicArm.getState() == true) {
            return false;
        }
        return true;
    }
    public void stopArm() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.telemetry.addLine("Stopping Arm");
    }
    public void extendArm(){
        setArmPower(1);
    }
    public void retractArm(){setArmPower(-1);}
}