package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="Testing")
public class TestOpMode extends NextFTCOpMode {
    public TestOpMode() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Turret.INSTANCE, Flywheel.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private PedroDriverControlled driverControlled;
    private double scalar = 1;

    private MainTeleOp.HeadingMode headingMode = MainTeleOp.HeadingMode.GAMEPAD;
    private double targetHeading;



    ControlSystem controller = ControlSystem.builder()
            .angular(AngleType.RADIANS,
                    feedback -> feedback.posPid(0.807, 0, 0.001)
            ).build();

    @Override
    public void onInit() {
        Turret.INSTANCE.disableTracking.schedule();
        Flywheel.INSTANCE.turnFlywheelOff.schedule();
        //LightingController.init();
    }

    @Override
    public void onWaitForStart() {
        Gamepads.gamepad1().rightStickX();
        PedroComponent.follower().setPose(new Pose(RobotState.AUTO_END_X, RobotState.AUTO_END_Y, RobotState.AUTO_END_HEADING));
        Pose robotPose = PedroComponent.follower().getPose();
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
            driverControlled = new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY(),
                    Gamepads.gamepad1().leftStickX(),
                    () -> {
                        switch (headingMode) {
                            case GAMEPAD:
                                return (double) (gamepad1.right_stick_x * -1);
                            case ABSOLUTE:
                                return controller.calculate(new KineticState(PedroComponent.follower().getHeading()));
                            default:
                                throw new UnsupportedOperationException("Unknown heading mode: " + headingMode);
                        }
                    },
                    false
            );
        } else {
            driverControlled = new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX().negate(),
                    () -> {
                        switch (headingMode) {
                            case GAMEPAD:
                                return (double) (gamepad1.right_stick_x * -1);
                            case ABSOLUTE:
                                return controller.calculate(new KineticState(PedroComponent.follower().getHeading()));
                            default:
                                throw new UnsupportedOperationException("Unknown heading mode: " + headingMode);
                        }
                    },
                    false
            );
        }

        driverControlled.schedule();

        Turret.INSTANCE.setTurretPosition(RobotState.TURRET_END_POS).schedule();
        Lift.INSTANCE.disengageLift.schedule();

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


        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    headingMode = MainTeleOp.HeadingMode.ABSOLUTE;
                    targetHeading = RobotState.GATE_HEADING;
                });

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> {
                    headingMode = MainTeleOp.HeadingMode.ABSOLUTE;
                    targetHeading = RobotState.PARK_HEADING;
                });

        Gamepads.gamepad1().rightStickX().greaterThan(0.05)
                .whenBecomesTrue(() -> headingMode = MainTeleOp.HeadingMode.GAMEPAD);
    }

    @Override
    public void onUpdate() {
        //driverControlled.update();
        //BindingManager.update();
        controller.setGoal(new KineticState(targetHeading));
        telemetry.addData("applied power", controller.calculate(new KineticState(PedroComponent.follower().getHeading())));
        telemetry.addData("headingerrorRadians", targetHeading - PedroComponent.follower().getHeading());
        telemetry.addData("targetHeadingDegrees", Math.toDegrees(targetHeading));
        telemetry.addData("currentHeadingDegrees", Math.toDegrees(PedroComponent.follower().getHeading()));
        telemetry.update();
    }

    public enum HeadingMode {
        GAMEPAD,
        ABSOLUTE,
    }
}
