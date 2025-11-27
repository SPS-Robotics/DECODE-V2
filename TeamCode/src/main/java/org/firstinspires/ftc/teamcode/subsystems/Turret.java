package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SoftwareConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }

    ControlSystem controller = ControlSystem.builder()
            .posPid(SoftwareConstants.Turret.kP, SoftwareConstants.Turret.kI, SoftwareConstants.Turret.kD)
            .build();

    private final MotorEx turretRotator = new MotorEx("turretRotator").zeroed().brakeMode();
    double turretAngleRadians, targetTicks;

    public double calculateTurretAngleToGoal() {
        Pose currentPose = PedroComponent.follower().getPose();

        double dx = SoftwareConstants.Poses.blueGoalPose.getX() - currentPose.getX();
        double dy = SoftwareConstants.Poses.blueGoalPose.getY() - currentPose.getY();

        double globalAngleToGoal = Math.atan2(dy, dx);

        return globalAngleToGoal - currentPose.getHeading();
    }

    public double convertTurretAngleToTicks(double turretAngleRadians) {
        double targetTicks = (turretAngleRadians / (2 * Math.PI)) * SoftwareConstants.Turret.ticksPerRevolution * SoftwareConstants.Turret.pulleyRatio;

        return MathUtils.clamp(targetTicks, SoftwareConstants.Turret.minTicks, SoftwareConstants.Turret.maxTicks);
    }

    public void periodic() {
        turretAngleRadians = calculateTurretAngleToGoal();
        targetTicks = convertTurretAngleToTicks(turretAngleRadians);
        controller.setGoal(new KineticState(targetTicks, 0, 0));

        turretRotator.setPower(
                controller.calculate(
                        turretRotator.getState()
                )
        );
    }
}
