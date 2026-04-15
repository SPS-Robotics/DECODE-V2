package org.firstinspires.ftc.teamcode.opmode.tuning;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.TuningFlywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.LightingController;

import dev.nextftc.core.commands.Command;
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

@TeleOp(name="TUNING - Flywheel")
public class FlywheelTuning extends NextFTCOpMode {
    public FlywheelTuning() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, TuningFlywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }
    public static double hoodPos = 0.4;
    double[] flywheelIncrements = {50, 10, 1};

    int hoodIncrementIndex = 0;
    int flywheelIncrementIndex = 0;

    @Override
    public void onInit() {
        Turret.INSTANCE.disableTracking.schedule();
        TuningFlywheel.INSTANCE.turnOff.schedule();
        LightingController.init();
        PedroComponent.follower().setPose(new Pose(RobotState.AUTO_END_X, RobotState.AUTO_END_Y, RobotState.AUTO_END_HEADING));
    }

    @Override
    public void onWaitForStart() {
        Gamepads.gamepad1().rightStickX();
        //PedroComponent.follower().setPose(new Pose(RobotState.AUTO_END_X, RobotState.AUTO_END_Y, RobotState.AUTO_END_HEADING));

        LightingController.get().update();
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

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(new ParallelGroup(
                        TuningFlywheel.INSTANCE.moveHoodByValue(0.02 + hoodPos),
                        new InstantCommand(() -> hoodPos += 0.02))
                );
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(new ParallelGroup(
                        TuningFlywheel.INSTANCE.moveHoodByValue(-0.02 + hoodPos),
                        new InstantCommand(() -> hoodPos -= 0.02))
                );
        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(TuningFlywheel.INSTANCE.moveFlywheelByValue(20));
        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(TuningFlywheel.INSTANCE.moveFlywheelByValue(-20));

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
                .whenBecomesTrue(TuningFlywheel.INSTANCE.turnOn)
                .whenBecomesFalse(TuningFlywheel.INSTANCE.turnOff);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(new SequentialGroup(
                        Intake.INSTANCE.openGate,
                        Intake.INSTANCE.intakeArtifacts
                ))
                .whenBecomesFalse(new ParallelGroup(
                        Intake.INSTANCE.closeGate,
                        Intake.INSTANCE.stopIntake
                ));

    }
    @Override
    public void onUpdate() {
        LightingController.get().update();

        Pose robotPose = follower().getPose();

        TuningFlywheel.INSTANCE.hoodServo.setPosition(hoodPos);
        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());
        telemetry.addData("Alliance", RobotState.ALLIANCE_COLOR);
        telemetry.addData("Goal Pose", RobotState.GOAL_POSE);
        telemetry.addData("TurretOffset", RobotState.TURRET_END_POS);
        telemetry.addData("Distance", robotPose.distanceFrom(RobotState.GOAL_POSE));
        telemetry.update();
        drawOnlyCurrent();
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
