package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes2019skystone;
import android.view.Display;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;


public class Lightning_Drivetrain {
    private DcMotor LMotor = null;
    private DcMotor RMotor = null;
    private Servo foundationServo = null;
    private Servo rightFoundationGrabber = null;
    private Servo leftFoundationGrabber = null;

    private LinearOpMode opMode = null;
    private boolean isHandled = false;
    private boolean isFast = true;

    public Lightning_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        //init motors
        LMotor = opMode.hardwareMap.get(DcMotor.class, "LMotor");
        RMotor = opMode.hardwareMap.get(DcMotor.class, "RMotor");
        LMotor.setDirection(DcMotor.Direction.FORWARD);
        RMotor.setDirection(DcMotor.Direction.REVERSE);
        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //don't forget that the brakes are on
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();
        setEncoderTargets(0, 0);

        //init servo
        foundationServo = opMode.hardwareMap.get(Servo.class, "foundationServo");
        foundationServo.setPosition(0);

        rightFoundationGrabber = opMode.hardwareMap.get(Servo.class, "rightFoundGrab");
        rightFoundationGrabber.setPosition(0);
        leftFoundationGrabber = opMode.hardwareMap.get(Servo.class, "leftFoundGrab");
        leftFoundationGrabber.setPosition(1);
    }

    public void unbrake(){
        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void driveWithSticks() {
        double leftPower = -opMode.gamepad1.left_stick_y;
        double rightPower = -opMode.gamepad1.right_stick_y;
        if (opMode.gamepad1.a && !isHandled) {
            isFast = !isFast;
            isHandled = true;
        } else if (!opMode.gamepad1.a && isHandled) {
            isHandled = false;
        }
        if(!isFast){
            leftPower = leftPower / 4;
            rightPower = rightPower / 4;
        }
        setMotorPowers(leftPower, rightPower);
    }

    public void controlFoundation(){
        if (opMode.gamepad2.a) {
            foundationServo.setPosition(1);
            rightFoundationGrabber.setPosition(1);
            leftFoundationGrabber.setPosition(0);
        } else {
            foundationServo.setPosition(0);
            rightFoundationGrabber.setPosition(0);
            leftFoundationGrabber.setPosition(1);
        }
    }

    public void resetEncoder() {
        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runWithEncoder();
    }

    public void runWithEncoder() {
        LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder() {
        LMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setEncoderTargets(int left, int right){
        LMotor.setTargetPosition(left);
        RMotor.setTargetPosition(right);
    }

    public void setMotorPowers(double leftPower, double rightPower){
        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);
    }
}