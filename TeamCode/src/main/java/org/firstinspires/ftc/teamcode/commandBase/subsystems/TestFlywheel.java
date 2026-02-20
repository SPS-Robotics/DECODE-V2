package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class TestFlywheel implements Subsystem {
    public static final TestFlywheel INSTANCE = new TestFlywheel();
    private TestFlywheel() { }

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("flywheelMotor1").floatMode(),
            new MotorEx("flywheelMotor2").reversed().floatMode()
    );

    private final ServoEx hoodServo = new ServoEx("hoodServo", 0.0001);

    double power = 0;
    boolean spinFlywheel = false;

    public Command turnFlywheelOn = new InstantCommand(() -> spinFlywheel = true);
    public Command turnFlywheelOff = new InstantCommand(() -> spinFlywheel = false);

    public void periodic() {
        if (spinFlywheel) power = 1;
        else power = 0;

        flywheelMotors.setPower(power);

        ActiveOpMode.telemetry().addData("Flywheel Speed", flywheelMotors.getVelocity());
        ActiveOpMode.telemetry().addData("Hood Position", hoodServo.getPosition());
    }
}
