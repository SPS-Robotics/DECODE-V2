package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import org.firstinspires.ftc.teamcode.globals.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Lift implements Subsystem {
    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    private final ServoEx leftServo = new ServoEx("leftServo");
    private final ServoEx rightServo = new ServoEx("rightServo");

    public Command engageLift = new ParallelGroup(
            new SetPosition(leftServo, Constants.Lift.LEFT_SERVO_ENGAGED),
            new SetPosition(rightServo, Constants.Lift.RIGHT_SERVO_ENGAGED)
    );

    public Command disengageLift = new ParallelGroup(
            new SetPosition(leftServo, Constants.Lift.LEFT_SERVO_DISENGAGED),
            new SetPosition(rightServo, Constants.Lift.RIGHT_SERVO_DISENGAGED)
    );
}
