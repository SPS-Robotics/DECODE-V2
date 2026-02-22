package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.util.InterpLUT;

import java.util.Arrays;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
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
            Arrays.asList(10.0, 11.0, 12.0, 13.0), // distance
            Arrays.asList(1600.0, 0.2, 0.3, 0.4) // shooter velocity
    );
    private final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(10.0, 11.0, 12.0, 13.0), // distance
            Arrays.asList(0.1, 0.2, 0.3, 0.4) // hood position
    );

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
