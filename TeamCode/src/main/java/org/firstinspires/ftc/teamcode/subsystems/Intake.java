package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.SoftwareConstants;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    List<BallColor> ballOrder = new ArrayList<>();
    private boolean isBallPresent = false;

    public enum BallColor {
        PURPLE, GREEN
    }

    public boolean isPurple(NormalizedRGBA reading) {
        return false;
    }

    public boolean isGreen(NormalizedRGBA reading) {
        return false;
    }

    public List<BallColor> getBallOrder() {
        return ballOrder;
    }

    public void clearIntake() {
        ballOrder.clear();
    }


    private final MotorEx intakeMotor = new MotorEx("intakeMotor").brakeMode();
    private final NormalizedColorSensor colorSensor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "colorSensor");

    private final ServoEx gateServo = new ServoEx("gateServo");

    public Command intakeArtifacts = new SetPower(intakeMotor, SoftwareConstants.Intake.intakePower);
    public Command outtakeArtifacts = new SetPower(intakeMotor, -SoftwareConstants.Intake.intakePower);
    public Command stopIntake = new SetPower(intakeMotor, 0);

    public Command openGate = new SetPosition(gateServo, SoftwareConstants.Intake.gateOpenPosition);
    public Command closeGate = new SetPosition(gateServo, SoftwareConstants.Intake.gateClosePosition);

    public void periodic() {
        NormalizedRGBA currentReading = colorSensor.getNormalizedColors();

        boolean isPurpleNow = isPurple(currentReading);
        boolean isGreenNow = isGreen(currentReading);
        boolean isBallPresentNow = isPurpleNow || isGreenNow;

        if (isBallPresentNow && !isBallPresent) {
            if (isPurpleNow) ballOrder.add(BallColor.PURPLE);
            else ballOrder.add(BallColor.GREEN);
        }

        isBallPresent = isBallPresentNow;

        ActiveOpMode.telemetry().addData("Balls", ballOrder);
    }
}
