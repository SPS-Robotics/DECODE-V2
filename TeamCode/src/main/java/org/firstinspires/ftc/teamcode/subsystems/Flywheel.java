package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SoftwareConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;

public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }

    Pose goalPose = SoftwareConstants.Poses.blueGoalPose;

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("flywheelMotor1").floatMode(),
            new MotorEx("flywheelMotor2").reversed().floatMode()
    );

    private final ServoGroup hoodServos = new ServoGroup(
            new ServoEx("hoodServo1"),
            new ServoEx("hoodServo2")
    );

    public double getHoodPosFromDistance(double distanceUnits) {
        // Replace with regression after testing.
        return 0.5;
    }

    public double getDistanceToGoal() {
        Pose currentPose = PedroComponent.follower().getPose();

        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }

    ControlSystem controller = ControlSystem.builder()
            .velPid(SoftwareConstants.Flywheel.kP, SoftwareConstants.Flywheel.kI, SoftwareConstants.Flywheel.kD)
            .basicFF(SoftwareConstants.Flywheel.kV, SoftwareConstants.Flywheel.kA, SoftwareConstants.Flywheel.kS)
            .build();

    public Command turnFlywheelOn = new RunToVelocity(controller, SoftwareConstants.Flywheel.shootingVelocity);
    public Command turnFlywheelOff = new RunToVelocity(controller, 0);

    public void periodic() {
        double distance = getDistanceToGoal();
        double hoodPos = getHoodPosFromDistance(distance);

        if (Math.abs(hoodPos - hoodServos.getPosition()) > 0.01) hoodServos.setPosition(hoodPos);

        flywheelMotors.setPower(
                controller.calculate(
                        flywheelMotors.getState()
                )
        );
    }
}
