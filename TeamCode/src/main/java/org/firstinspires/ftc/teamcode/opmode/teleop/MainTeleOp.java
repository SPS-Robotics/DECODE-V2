package org.firstinspires.ftc.teamcode.opmode.teleop;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.LightingController;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    public MainTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Turret.INSTANCE, Flywheel.INSTANCE, Limelight.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private double distanceOffset = 0;
    private double lateralOffset = 0;

    private MecanumDriverControlled driverControlled;
    private double scalar = 1;
    private HeadingMode headingMode = HeadingMode.GAMEPAD;
    private double targetHeading;

    private final MotorEx frontLeft = new MotorEx("frontLeft").brakeMode();
    private final MotorEx frontRight = new MotorEx("frontRight").brakeMode();
    private final MotorEx backLeft = new MotorEx("backLeft").brakeMode();
    private final MotorEx backRight = new MotorEx("backRight").brakeMode();

    ControlSystem controller = ControlSystem.builder()
            .angular(AngleType.RADIANS,
                    feedback -> feedback.posPid(0.85, 0, 0.002)
            ).build();

    @Override
    public void onInit() {
        Turret.INSTANCE.disableTracking.schedule();
        Flywheel.INSTANCE.turnFlywheelOff.schedule();
        Intake.INSTANCE.closeGate.schedule();
        LightingController.init();
        RobotState.SOTM = false;
        follower().setPose(new Pose(RobotState.AUTO_END_X, RobotState.AUTO_END_Y, RobotState.AUTO_END_HEADING));
    }

    @Override
    public void onWaitForStart() {
        Gamepads.gamepad1().rightStickX();

        LightingController.get().update();
        Pose robotPose = follower().getPose();
        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());
        telemetry.addData("Alliance", RobotState.ALLIANCE_COLOR);
        telemetry.addData("TurretOffset", RobotState.TURRET_END_POS);
        telemetry.addData("Goal Pose", RobotState.GOAL_POSE);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        if (RobotState.ALLIANCE_COLOR == RobotState.AllianceColor.BLUE) {
            driverControlled = new MecanumDriverControlled(
                    frontLeft,
                    frontRight,
                    backLeft,
                    backRight,
                    Gamepads.gamepad1().leftStickY().map(x -> Math.pow(x, 2) * Math.signum(x)),
                    Gamepads.gamepad1().leftStickX().negate().map(x -> Math.pow(x, 2) * Math.signum(x)),
                    () -> {
                        switch (headingMode) {
                            case GAMEPAD:
                                return Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
                            case ABSOLUTE:
                                return -controller.calculate(new KineticState(follower().getHeading()));
                            default:
                                throw new UnsupportedOperationException("Unknown heading mode: " + headingMode);
                        }
                    },
                    new FieldCentric(() -> Angle.fromRad(follower().getHeading())));
        } else {
            driverControlled = new MecanumDriverControlled(
                    frontLeft,
                    frontRight,
                    backLeft,
                    backRight,
                    Gamepads.gamepad1().leftStickY().negate().map(x -> Math.pow(x, 2) * Math.signum(x)),
                    Gamepads.gamepad1().leftStickX().map(x -> Math.pow(x, 2) * Math.signum(x)),
                    () -> {
                        switch (headingMode) {
                            case GAMEPAD:
                                return (-1) * Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
                            case ABSOLUTE:
                                return -controller.calculate(new KineticState(follower().getHeading()));
                            default:
                                throw new UnsupportedOperationException("Unknown heading mode: " + headingMode);
                        }
                    },
                    new FieldCentric(() -> Angle.fromRad(follower().getHeading())));
        }

        driverControlled.schedule();

        Turret.INSTANCE.setTurretPosition(RobotState.TURRET_END_POS).schedule();

        // Intake Controls
        Gamepads.gamepad1().rightTrigger().greaterThan(0.1)
                .whenBecomesTrue(Intake.INSTANCE.intakeArtifacts)
                .whenBecomesFalse(Intake.INSTANCE.stopIntake);

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(Intake.INSTANCE.outtakeArtifacts)
                .whenBecomesFalse(Intake.INSTANCE.stopIntake);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(new SequentialGroup(
                        Intake.INSTANCE.openGate,
                        Intake.INSTANCE.intakeArtifacts
                ))
                .whenBecomesFalse(new ParallelGroup(
                        Intake.INSTANCE.stopIntake,
                        Intake.INSTANCE.closeGate
                ));

        // Shooter Controls
        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Turret.INSTANCE.enableTracking)
                .whenBecomesFalse(Turret.INSTANCE.disableTracking);

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Flywheel.INSTANCE.turnFlywheelOn)
                .whenBecomesFalse(Flywheel.INSTANCE.turnFlywheelOff);

        //Drive Controls
        Gamepads.gamepad1().rightStickX().greaterThan(0.05).or(Gamepads.gamepad1().rightStickX().lessThan(-0.05))
                .whenBecomesTrue(() -> headingMode = HeadingMode.GAMEPAD);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.05)
                .whenBecomesTrue(() -> scalar = 0.3)
                .whenBecomesFalse(() -> scalar = 1);

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    headingMode = HeadingMode.ABSOLUTE;
                    targetHeading = RobotState.GATE_HEADING;
                });

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    headingMode = HeadingMode.ABSOLUTE;
                    targetHeading = RobotState.PARK_HEADING;
                });

        // Debug Controls
        Gamepads.gamepad2().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Flywheel.INSTANCE.enableDistanceOverride)
                .whenBecomesFalse(Flywheel.INSTANCE.disableDistanceOverride);

        Gamepads.gamepad2().square()
                .whenBecomesTrue(() -> follower().setPose(RobotState.GATE_RELOC_POSE));

        Gamepads.gamepad2().cross()
                .whenBecomesTrue(() -> follower().setPose(RobotState.LOADING_ZONE));

        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(() -> {
                    RobotState.GOAL_POSE.plus(new Pose(-1, 1));
                    distanceOffset += 1;
                });

        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(() -> {
                    RobotState.GOAL_POSE.plus(new Pose(1, -1));
                    distanceOffset -= 1;
                });

        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(() -> {
                    RobotState.GOAL_POSE.plus(new Pose(-1, 0));
                    lateralOffset -= 1;
                });

        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(() -> {
                    RobotState.GOAL_POSE.plus(new Pose(1, 0));
                    lateralOffset += 1;
                });
    }

    @Override
    public void onUpdate() {
        driverControlled.setScalar(scalar);

        controller.setGoal(new KineticState(targetHeading));

        LightingController.get().update();

        Pose robotPose = follower().getPose();
        telemetry.addData("Robot X", "%.2f", robotPose.getX());
        telemetry.addData("Robot Y", "%.2f", robotPose.getY());
        telemetry.addData("Robot Heading", "%.2f", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Alliance", RobotState.ALLIANCE_COLOR);
        telemetry.addData("Goal Pose", RobotState.velocityCompensate(RobotState.GOAL_POSE));
        telemetry.addData("Distance Offset (in)", distanceOffset);
        telemetry.addData("Lateral Offset L/R (in)", lateralOffset);
        telemetry.update();
    }

    @Override
    public void onStop() { }

    public enum HeadingMode {
        GAMEPAD,
        ABSOLUTE,
    }
}
