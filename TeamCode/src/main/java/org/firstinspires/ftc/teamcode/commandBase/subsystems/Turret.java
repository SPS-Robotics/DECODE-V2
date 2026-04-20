package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.util.InterpLUT;
import org.firstinspires.ftc.teamcode.util.MathUtils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.globals.Constants;

import java.util.Arrays;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }

    ControlSystem controller = ControlSystem.builder()
            .posPid(Constants.Turret.kP, Constants.Turret.kI, Constants.Turret.kD)
            .build();

    private final MotorEx turretRotator = new MotorEx("turretRotator").brakeMode();

    private TouchSensor magneticLimitSwitch;

    @Override
    public void initialize() {
        magneticLimitSwitch = ActiveOpMode.hardwareMap().get(TouchSensor.class, "magneticLimitSwitch");
    }
    private boolean turretTracking = false;

    private boolean debugMode = false;
    private double debugTarget = 0;

    public double getTurretPosition() {
        return turretRotator.getCurrentPosition();
    }



    public double calculateTurretPosition(Pose goalPose) {
        Pose robotPose = PedroComponent.follower().getPose();
        Pose turretPose = MathUtils.translatePose(robotPose, -Constants.Turret.CENTRE_OFFSET);
        double turretAngleRadians = MathUtils.calculateAngleToPose(turretPose, goalPose);

        double goalLocation = (turretAngleRadians / (2 * Math.PI)) * Constants.Turret.ticksPerRevolution * Constants.Turret.pulleyRatio;
        return MathUtils.clampValue(goalLocation, Constants.Turret.MIN_TICKS, Constants.Turret.MAX_TICKS);
    }

    public Command enableTracking = new InstantCommand(() -> turretTracking = true);
    public Command disableTracking = new InstantCommand(() -> turretTracking = false);

    public Command moveTurretBy(double pos) {
        return new InstantCommand(() -> {
            debugTarget += pos;
            debugMode = true;
        });
    }

    public Command setTurretPosition(double pos) {
        return new InstantCommand(() -> {
            turretRotator.setCurrentPosition(pos);
            debugMode = false;
            debugTarget = 0;
        });
    }

    @Override
    public void periodic() {
        //if (magneticLimitSwitch.isPressed()) turretRotator.setCurrentPosition(Constants.Turret.RELOC_POS);
        double targetPos;
        if (debugMode) targetPos = debugTarget;
        else {
            if (RobotState.SOTM) targetPos = calculateTurretPosition(RobotState.velocityCompensate(RobotState.GOAL_POSE));
            else targetPos = calculateTurretPosition(RobotState.GOAL_POSE);
        }

        controller.setGoal(new KineticState(targetPos, 0, 0));

        double power;
        power = controller.calculate(turretRotator.getState());

        power += -PedroComponent.follower().getAngularVelocity() * Constants.Turret.angularVelocitykV;

        if (!turretTracking) power = 0;

        turretRotator.setPower(power);

        ActiveOpMode.telemetry().addData("TurretPos", turretRotator.getCurrentPosition());
        ActiveOpMode.telemetry().addData("TurretTarget", targetPos);
        ActiveOpMode.telemetry().addData("power applied", power);
        ActiveOpMode.telemetry().addData("switch pressed", magneticLimitSwitch.isPressed());
    }
}
