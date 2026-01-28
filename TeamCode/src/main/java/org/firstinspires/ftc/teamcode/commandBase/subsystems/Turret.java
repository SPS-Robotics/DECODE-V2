package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.util.MathUtils;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.globals.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }

    public static PIDCoefficients turretPID = new PIDCoefficients(Constants.Turret.kP, Constants.Turret.kI, Constants.Turret.kD);

    ControlSystem controller = ControlSystem.builder()
            .posPid(turretPID)
            .build();

    private final MotorEx turretRotator = new MotorEx("turretRotator").zeroed().brakeMode();
    private boolean turretTracking = true;
    private double power = 0;

    public Command toggleTracking = new InstantCommand(() -> turretTracking = !turretTracking);


    public double calculateTurretPosition() {
        Pose robotPose = PedroComponent.follower().getPose();
        double turretAngleRadians = MathUtils.calculateAngleToPose(robotPose, RobotState.GOAL_POSE);

        double tickOffset = (turretAngleRadians / (2 * Math.PI)) * Constants.Turret.ticksPerRevolution * Constants.Turret.pulleyRatio;

        return MathUtils.clampValue(turretRotator.getCurrentPosition() + tickOffset, Constants.Turret.MIN_TICKS, Constants.Turret.MAX_TICKS);
    }



    public void periodic() {
        double targetPos = calculateTurretPosition();

        controller.setGoal(new KineticState(targetPos, 0, 0));

        if (!turretTracking) power = 0;
        else power = controller.calculate(turretRotator.getState());

        turretRotator.setPower(power);

    }
}
