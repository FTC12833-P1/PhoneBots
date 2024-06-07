package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Archimedes_WobbleScorer {
    private enum WobblePositions{
        START,
        CARRY,
        GATHER,
        OTHER
    }

    private LinearOpMode opMode;

    private DcMotorEx wobbleArm;
    private DigitalChannel topStop;
    private DigitalChannel bottomStop;
    private Servo schmacker;
    private Servo holder;

    private WobblePositions selectedPosition = WobblePositions.START;
    private WobblePositions currentPosition = WobblePositions.START;
    private boolean armIsHandled = false;

    private final static int CARRY_POSITION = 1310;

    public Archimedes_WobbleScorer(LinearOpMode opMode){

        this.opMode = opMode;

        wobbleArm = opMode.hardwareMap.get(DcMotorEx.class, "wobbleMechanism");
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topStop = opMode.hardwareMap.get(DigitalChannel.class,"topStop");
        bottomStop = opMode.hardwareMap.get(DigitalChannel.class,"bottomStop");

        schmacker = opMode.hardwareMap.get(Servo.class,"weeble_schmacker");
        holder = opMode.hardwareMap.get(Servo.class,"weeble_holder ");

        unschmack();
        hold();

        moveToStart();
    }

    public void controlWobble(){
        if(opMode.gamepad2.left_trigger > .01){
            schmack();
        }else{
            unschmack();
        }

        if (opMode.gamepad2.left_bumper){
            deliver(false);
        }

        if(opMode.gamepad2.left_stick_y < 0 && !armIsHandled){
            selectedPosition = WobblePositions.CARRY;
            currentPosition = WobblePositions.OTHER;
            wobbleArm.setTargetPosition(CARRY_POSITION);
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            carry();
            armIsHandled = true;
        }else if (opMode.gamepad2.left_stick_y > 0 && !armIsHandled){
            selectedPosition = WobblePositions.GATHER;
            currentPosition = WobblePositions.OTHER;
            wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lower();
            armIsHandled = true;
        }else if(opMode.gamepad2.left_stick_button && !armIsHandled){
            selectedPosition = WobblePositions.START;
            currentPosition = WobblePositions.OTHER;
            wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift();
            armIsHandled = true;
        }else {
            armIsHandled = false;
        }

        if(currentPosition != selectedPosition) {
            switch (selectedPosition) {
                case START:
                    if(isTriggered(topStop)){
                        stop();
                        currentPosition = WobblePositions.START;
                        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    break;
                case CARRY:
                    currentPosition = WobblePositions.CARRY;
                    break;
                case GATHER:
                    if(isTriggered(bottomStop)){
                        stop();
                        currentPosition = WobblePositions.GATHER;
                    }
                    break;
            }
        }
    }

    public void deliver(boolean firstDeliver){
        if(firstDeliver) {
            moveToGather();
            unhold();
            opMode.sleep(500);
        }else{
            moveToScore();
            opMode.sleep(500);
        }
        schmack();
        moveToStart();
        unschmack();
    }

    public void moveToStart(){
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift();

        while(!isTriggered(topStop) && opMode.opModeIsActive()){
        }

        stop();
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveToGather(){
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lower();

        while(!isTriggered(bottomStop) && opMode.opModeIsActive()){
        }

        stop();
    }

    public void moveToScore(){
        wobbleArm.setTargetPosition(CARRY_POSITION + 300);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carry();
    }


    public void schmack() {
        schmacker.setPosition(0);
    }

    public void unschmack(){
        schmacker.setPosition(1);
    }

    private void hold() {
        holder.setPosition(1);
    }

    private void unhold() {
        holder.setPosition(0);
    }

    private boolean isTriggered(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    }

    public void lift(){
        wobbleArm.setPower(-.25);
    }
    public void lower(){
        wobbleArm.setPower(.35);
    }
    public void stop(){
        wobbleArm.setPower(0);
    }
    public void carry(){
        wobbleArm.setPower(1);
    }
}