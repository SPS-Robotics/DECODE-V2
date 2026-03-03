package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.globals.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Lift implements Subsystem {
    public static final Lift INSTANCE = new Lift();

    private Lift() {
    }

    private final MotorEx backLeft = new MotorEx("backLeft").brakeMode();
    private final MotorEx backRight = new MotorEx("backRight").brakeMode();
    private final MotorEx frontLeft = new MotorEx("frontLeft").brakeMode().reversed();
    private final MotorEx frontRight = new MotorEx("frontRight").brakeMode().reversed();

    private final MotorGroup backMotors = new MotorGroup(backLeft, backRight);

    private final MotorGroup frontMotors = new MotorGroup(frontLeft, frontRight);

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

    public Command engageClutch = new ParallelGroup(
            new SetPower(backMotors, Constants.Lift.ENGAGING_POWER),
            new SetPower(frontMotors, -Constants.Lift.ENGAGING_POWER * 1.03)
    );

    public Command startLift = new ParallelGroup(
            new SetPower(backMotors, Constants.Lift.LIFT_POWER),
            new SetPower(frontMotors, -Constants.Lift.LIFT_POWER)
    );

    public Command stopLift = new ParallelGroup(
            new SetPower(backMotors, 0),
            new SetPower(frontMotors, 0)
    );

    public Command liftRobot = new SequentialGroup(
            engageLift,
            engageClutch,
            new Delay(Constants.Lift.ENGAGE_TIME),
            startLift
    );

    @Override
    public void periodic() { }
}
