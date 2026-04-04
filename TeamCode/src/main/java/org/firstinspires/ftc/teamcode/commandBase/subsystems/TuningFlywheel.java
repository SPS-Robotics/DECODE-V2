package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class TuningFlywheel implements Subsystem {
    public static final TuningFlywheel INSTANCE = new TuningFlywheel();
    private TuningFlywheel() { }

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("flywheelMotor1").reversed().floatMode(),
            new MotorEx("flywheelMotor2").floatMode()
    );

    public ServoEx hoodServo = new ServoEx("hoodServo", 0.0001);

    ControlSystem controller = ControlSystem.builder()
            .velPid(0.0014,0,0)
            .basicFF(0.000379,0,0.05)
            .build();

    double power;
    double flywheelTarget = 2000;
    private boolean shoot = false;

    public Command moveFlywheelByValue(double increment) {
        return new InstantCommand(() -> {
            flywheelTarget += increment;
            controller.setGoal(new KineticState(0, flywheelTarget, 0));
        });
    }

    public Command moveHoodByValue(double newPosition) {
        return new SetPosition(hoodServo, newPosition);
    }

    public Command turnOn = new InstantCommand(() -> shoot = true);
    public Command turnOff = new InstantCommand(() -> shoot = false);

    public double hoodPosition = 0.4;

    @Override
    public void initialize() {
        hoodServo.setPosition(hoodPosition);
        flywheelTarget = 0;
    }

    @Override
    public void periodic() {
        controller.setGoal(new KineticState(0, flywheelTarget, 0));
        if (shoot) power = controller.calculate(flywheelMotors.getState());
        else power = 0;

        hoodPosition = hoodServo.getPosition();

        flywheelMotors.setPower(power);

        ActiveOpMode.telemetry().addData("Flywheel Speed", flywheelMotors.getVelocity());
        ActiveOpMode.telemetry().addData("Flywheel Target", flywheelTarget);
        ActiveOpMode.telemetry().addData("Applied Power", power);
        ActiveOpMode.telemetry().addData("Hood Position", hoodServo.getPosition());
    }
}
