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



    private boolean spinFlywheel = false;
    public boolean atSpeed = false;

    private boolean distanceOverride = false;
    double power;

    ControlSystem controller = ControlSystem.builder()
            .velPid(Constants.Flywheel.kP, Constants.Flywheel.kI, Constants.Flywheel.kD)
            .basicFF(Constants.Flywheel.kV, Constants.Flywheel.kA, Constants.Flywheel.kS)
            .build();

    public Command turnFlywheelOn = new InstantCommand(() -> spinFlywheel = true);
    public Command turnFlywheelOff = new InstantCommand(() -> spinFlywheel = false);

    public Command enableDistanceOverride = new InstantCommand(() -> distanceOverride = true);
    public Command disableDistanceOverride = new InstantCommand(() -> distanceOverride = false);

    public void periodic() {
        Pose robotPose = PedroComponent.follower().getPose();

        double distance = robotPose.distanceFrom(RobotState.GOAL_POSE);
        if (distanceOverride) distance = 50;

        hoodServo.setPosition(RobotState.hoodLUT.get(distance));
        controller.setGoal(new KineticState(0, RobotState.velocityLUT.get(distance), 0));

        if (!spinFlywheel) power = 0;
        else power = controller.calculate(flywheelMotors.getState());
        flywheelMotors.setPower(power);

        atSpeed = Math.abs(controller.getGoal().getVelocity() - flywheelMotors.getVelocity()) < 50;

        ActiveOpMode.telemetry().addData("Flywheel Speed", flywheelMotors.getVelocity());
        ActiveOpMode.telemetry().addData("Flywheel Error", controller.getGoal().getVelocity() - flywheelMotors.getVelocity());
    }
}
