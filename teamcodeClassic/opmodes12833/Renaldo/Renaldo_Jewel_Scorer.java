package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Renaldo_Jewel_Scorer {

    private Servo jewelArm = null;
    private ColorSensor colorSensor = null;
    private double currentArmPosition;
    private ElapsedTime runtime = new ElapsedTime();

    private Renaldo_OpMode opMode;

    final double START_POSITION = .15;

    final double MOVE_POSITION_PART1 = 0.4;
    final double MOVE_POSITION_PART2 = 0.75;

    public Renaldo_Jewel_Scorer(Renaldo_OpMode opMode) { //Constructor
        this.opMode = opMode;

        jewelArm = opMode.hardwareMap.servo.get("jewel_arm");
        jewelArm.setPosition(START_POSITION);
        currentArmPosition = START_POSITION;

        colorSensor = opMode.hardwareMap.colorSensor.get("sensor_color_distance");
        colorSensor.enableLed(true);
    }

    public int getLeftColor() {
        int jewelColor = opMode.NOTHING;
        if (colorSensor.red() > colorSensor.blue()) {
            jewelColor = opMode.RED;
        } else if (colorSensor.blue() > colorSensor.red()) {
            jewelColor = opMode.BLUE;
        }
        opMode.telemetry.addData("Left Jewel", (jewelColor == 1)? "Red": "Blue");
        return jewelColor;
    }

    public void toggle() {
        if (currentArmPosition == START_POSITION)
            lower();
        else
            raise();
    }

    public void raise() {
        jewelArm.setPosition(START_POSITION);
        currentArmPosition = START_POSITION;
        opMode.sleep(2000);
    }

    public void lower() {
        jewelArm.setPosition(MOVE_POSITION_PART1);
        opMode.sleep(500);
        jewelArm.setPosition(MOVE_POSITION_PART2);
        opMode.sleep(300);
        currentArmPosition = MOVE_POSITION_PART2;
    }
}

