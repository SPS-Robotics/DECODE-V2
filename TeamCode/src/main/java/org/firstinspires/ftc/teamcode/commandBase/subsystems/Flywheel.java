package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.util.InterpLUT;

import java.util.Arrays;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("flywheelMotor1").floatMode(),
            new MotorEx("flywheelMotor2").reversed().floatMode()
    );

    private final ServoEx hoodServo = new ServoEx("hoodServo", 0.0001);

    private final InterpLUT velocityLUT = new InterpLUT(
            Arrays.asList(42.0971, 49.6647, 56.9606, 66.5632, 67.5254, 72.2892, 80.6825, 87.8641), // Distance
            Arrays.asList(1250.0, 1250.0, 1350.0, 1400.0, 1450.0, 1450.0, 1500.0, 1600.0) // Flywheel RPM
    ).createLUT();

    private final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(42.0971, 49.6647, 56.9606, 66.5632, 67.5254, 72.2892, 80.6825, 87.8641),
            Arrays.asList(0.32, 0.30, 0.24, 0.20, 0.34, 0.20, 0.26, 0.18)
    ).createLUT();

    private boolean spinFlywheel = false;
    double power;

    ControlSystem controller = ControlSystem.builder()
            .velPid(Constants.Flywheel.kP, Constants.Flywheel.kI, Constants.Flywheel.kD)
            .basicFF(Constants.Flywheel.kV, Constants.Flywheel.kA, Constants.Flywheel.kS)
            .build();

    public Command turnFlywheelOn = new InstantCommand(() -> spinFlywheel = true);
    public Command turnFlywheelOff = new InstantCommand(() -> spinFlywheel = false);

    public void periodic() {
        Pose robotPose = PedroComponent.follower().getPose();

        double distance = robotPose.distanceFrom(RobotState.GOAL_POSE);

        hoodServo.setPosition(hoodLUT.get(distance));
        controller.setGoal(new KineticState(0, velocityLUT.get(distance), 0));

        if (!spinFlywheel) power = 0;
        else power = controller.calculate(flywheelMotors.getState());

        flywheelMotors.setPower(power);

        ActiveOpMode.telemetry().addData("Flywheel Speed", flywheelMotors.getVelocity());
        ActiveOpMode.telemetry().addData("Flywheel Error", controller.getGoal().getVelocity() - flywheelMotors.getVelocity());
    }
}
