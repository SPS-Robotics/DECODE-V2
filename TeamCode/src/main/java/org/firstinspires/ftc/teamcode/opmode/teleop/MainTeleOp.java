package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
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
                new SubsystemComponent(Intake.INSTANCE, Flywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private final MotorEx frontLeft = new MotorEx("frontLeft").brakeMode().zeroed().reversed();
    private final MotorEx frontRight = new MotorEx("frontRight").brakeMode().zeroed();
    private final MotorEx backLeft = new MotorEx("backLeft").brakeMode().zeroed().reversed();
    private final MotorEx backRight = new MotorEx("backRight").brakeMode().zeroed();

    private final IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    @Override
    public void onInit() {
        PedroComponent.follower().setPose(RobotState.AUTO_END_POSE);
        Gamepads.gamepad1().rightStickX();
    }

    @Override
    public void onWaitForStart() { }

    @Override
    public void onStartButtonPressed() {
        /*
        Command driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
        */

        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                new FieldCentric(imu)
        );

        driverControlled.schedule();

        Gamepads.gamepad1().rightTrigger().greaterThan(0.05)
                .whenBecomesTrue(Intake.INSTANCE.intakeArtifacts)
                .whenBecomesFalse(Intake.INSTANCE.stopIntake);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.05)
                .whenBecomesTrue(Intake.INSTANCE.outtakeArtifacts)
                .whenBecomesFalse(Intake.INSTANCE.stopIntake);

        Gamepads.gamepad1().triangle()
                .whenBecomesTrue(Turret.INSTANCE.toggleTracking);

        Gamepads.gamepad1().leftBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Flywheel.INSTANCE.firingSpeed)
                .whenBecomesFalse(Flywheel.INSTANCE.lowSpeed);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(new ParallelGroup(
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
        telemetry.addData("Robot Pose", PedroComponent.follower().getPose());
    }
    @Override
    public void onStop() { }
}
