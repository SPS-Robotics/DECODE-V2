package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import org.firstinspires.ftc.teamcode.globals.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private final MotorEx intakeMotor = new MotorEx("intakeMotor").brakeMode();

    private final ServoEx gateServo = new ServoEx("gateServo");

    public Command intakeArtifacts = new SetPower(intakeMotor, Constants.Intake.INTAKE_POWER).requires(intakeMotor);
    public Command outtakeArtifacts = new SetPower(intakeMotor, -Constants.Intake.INTAKE_POWER).requires(intakeMotor);
    public Command stopIntake = new SetPower(intakeMotor, 0).requires(intakeMotor);

    public Command openGate = new SetPosition(gateServo, Constants.Intake.GATE_OPEN).requires(gateServo);
    public Command closeGate = new SetPosition(gateServo, Constants.Intake.GATE_CLOSE).requires(gateServo);
}
