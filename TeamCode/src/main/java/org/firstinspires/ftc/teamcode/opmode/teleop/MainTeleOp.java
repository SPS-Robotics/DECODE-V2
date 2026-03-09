package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.TuningFlywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Drawing;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
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
    @Override
    public void onInit() {
        Turret.INSTANCE.disableTracking.schedule();
        Flywheel.INSTANCE.turnFlywheelOff.schedule();
    }

    @Override
    public void onWaitForStart() {
        Gamepads.gamepad1().rightStickX();
        PedroComponent.follower().setPose(RobotState.AUTO_END_POSE);
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
        Command driverControlled;
        if (RobotState.ALLIANCE_COLOR == RobotState.AllianceColor.BLUE) {
            driverControlled = new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX().negate(),
                    false
            );
        }
        else {
            driverControlled = new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX().negate(),
                    Gamepads.gamepad1().rightStickX().negate(),
                    false
            );
        }

        driverControlled.schedule();

        //Flywheel.INSTANCE.turnFlywheelOn.schedule();
        Turret.INSTANCE.setTurretPosition(RobotState.TURRET_END_POS).schedule();
        Lift.INSTANCE.disengageLift.schedule();
        //Turret.INSTANCE.enableTracking.schedule();

        Gamepads.gamepad1().rightTrigger().greaterThan(0.05)
                .whenBecomesTrue(Intake.INSTANCE.intakeArtifacts)
                .whenBecomesFalse(Intake.INSTANCE.stopIntake);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.05)
                .whenBecomesTrue(Intake.INSTANCE.outtakeArtifacts)
                .whenBecomesFalse(Intake.INSTANCE.stopIntake);

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Turret.INSTANCE.enableTracking)
                .whenBecomesFalse(Turret.INSTANCE.disableTracking);

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Flywheel.INSTANCE.turnFlywheelOn)
                .whenBecomesFalse(Flywheel.INSTANCE.turnFlywheelOff);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(new SequentialGroup(
                        Intake.INSTANCE.openGate,
                        Intake.INSTANCE.intakeArtifacts
                ))
                .whenBecomesFalse(new ParallelGroup(
                        Intake.INSTANCE.stopIntake,
                        Intake.INSTANCE.closeGate
                ));

        Gamepads.gamepad1().square()
                .whenBecomesTrue(new SequentialGroup(
                        new InstantCommand(() -> PedroComponent.follower().breakFollowing()),
                        new InstantCommand(driverControlled::cancel),
                        Lift.INSTANCE.liftRobot
                ))
                .whenBecomesFalse(Lift.INSTANCE.stopLift);

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(Limelight.INSTANCE.relocaliseOdometry);

        Gamepads.gamepad2().cross()
                .whenBecomesTrue(Turret.INSTANCE.setTurretPosition(0));

        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(new InstantCommand(() -> RobotState.GOAL_POSE.plus(new Pose(-1, 1))));

        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(new InstantCommand(() -> RobotState.GOAL_POSE.minus(new Pose(-1, 1))));
    }

    @Override
    public void onUpdate() {
        Pose robotPose = follower().getPose();
        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());
        telemetry.addData("Alliance", RobotState.ALLIANCE_COLOR);
        telemetry.addData("Goal Pose", RobotState.GOAL_POSE);
        telemetry.addData("TurretOffset", RobotState.TURRET_END_POS);
        telemetry.addData("Distance", robotPose.distanceFrom(RobotState.GOAL_POSE));
        telemetry.update();
        //drawOnlyCurrent();
    }

    @Override
    public void onStop() { }

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(PedroComponent.follower().getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
}
