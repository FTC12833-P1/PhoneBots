package org.firstinspires.ftc.PhoneBots.teamcodeClassic.opmodes12833.Archimedes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Archimedes_Collector {

    private LinearOpMode opMode;

    private DcMotor intake1;
    private DcMotor intake2;
    private CRServo transportBungee;
    private CRServo stageWheel;

    private DistanceSensor sensorRing1;
    private DistanceSensor sensorRing2;
    private DistanceSensor sensorRing3;
    private DistanceUnit distanceUnitMM = DistanceUnit.MM;

    public Archimedes_Collector(LinearOpMode opMode) {
        this.opMode = opMode;

        intake1 = opMode.hardwareMap.get(DcMotor.class, "collector");
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2 = opMode.hardwareMap.get(DcMotor.class, "transport");
        intake2.setDirection(DcMotor.Direction.FORWARD);
        transportBungee = opMode.hardwareMap.get(CRServo.class, "firstStage");
        stageWheel = opMode.hardwareMap.get(CRServo.class, "secondStage");

        sensorRing1 = opMode.hardwareMap.get(DistanceSensor.class, "ringStage1");
        sensorRing2 = opMode.hardwareMap.get(DistanceSensor.class, "ringStage2");
        sensorRing3 = opMode.hardwareMap.get(DistanceSensor.class, "ringStage3");
    }

    public void controlTransport() {

        if (opMode.gamepad2.a) {
            autoTransport();
        }else{
            manualTransport();
        }
    }

    public void manualTransport() {
        transportBungee.setPower(0);
        stageWheel.setPower(0);
        if (opMode.gamepad2.b) {
            intake1.setPower(1);
            intake2.setPower(1);
        }else if (opMode.gamepad2.x){
            transportBungee.setPower(-1);
            stageWheel.setPower(1);
        }
        else {
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }

    public int autoTransport() {
        int numberOfRings = ringsDetected();

        if (numberOfRings == 0) {
            setTransportPowers(-1, 1, -1);
        } else if (numberOfRings == 1) {
            setTransportPowers(-1, 1, 0);
        } else if (numberOfRings == 2) {
            setTransportPowers(-1, 0, 0);
        } else {
            setTransportPowers(0, 0, 0);
        }

        return numberOfRings;
    }

    public void setTransportPowers(int intakePower, int bungeePower, int stageWheelPower) {
        intake1.setPower(intakePower);
        intake2.setPower(intakePower);
        transportBungee.setPower(bungeePower);
        stageWheel.setPower(stageWheelPower);
    }

    public void runStageWheel(){
        stageWheel.setPower(-1);
    }

    public void stopStageWheel(){
        stageWheel.setPower(0);
    }

    public int ringsDetected() {
//        opMode.telemetry.addData("sensor 1", String.format("%.01f", ringStage1.getDistance(distanceUnit)));
//        opMode.telemetry.addData("sensor 2", String.format("%.01f", ringStage2.getDistance(distanceUnit)));
//        opMode.telemetry.addData("sensor 3", String.format("%.01f", ringStage3.getDistance(distanceUnit)));

        if (sensorRing1.getDistance(distanceUnitMM) < 45) {  // at least 1
            if (sensorRing2.getDistance(distanceUnitMM) < 35) {  // at least 2
                if (sensorRing3.getDistance(distanceUnitMM) < 29) {  // 3
                    opMode.telemetry.addLine("three rings");
                    return 3;
                } else {
                    opMode.telemetry.addLine("two rings");
                    return 2;
                }
            } else {
                opMode.telemetry.addLine("one ring");
                return 1;
            }
        } else {
            opMode.telemetry.addLine("zero rings");
            return 0;
        }
    }
}
