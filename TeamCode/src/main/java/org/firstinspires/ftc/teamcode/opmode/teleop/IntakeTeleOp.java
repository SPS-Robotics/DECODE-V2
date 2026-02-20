package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.TestFlywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.utility.InstantCommand;
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

@TeleOp(name="Intake Test")
public class IntakeTeleOp extends NextFTCOpMode {

    public IntakeTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, TestFlywheel.INSTANCE)
        );
    }

    private final MotorEx frontLeft = new MotorEx("frontLeft").brakeMode().zeroed();
    private final MotorEx frontRight = new MotorEx("frontRight").brakeMode().zeroed().reversed();
    private final MotorEx backLeft = new MotorEx("backLeft").brakeMode().zeroed();
    private final MotorEx backRight = new MotorEx("backRight").brakeMode().zeroed().reversed();

    private final IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    public void onInit() {
        Gamepads.gamepad1().rightStickY();
    }

    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );

        driverControlled.schedule();



        Gamepads.gamepad1().rightTrigger().greaterThan(0.1)
                .whenBecomesTrue(Intake.INSTANCE.intakeArtifacts)
                .whenBecomesFalse(Intake.INSTANCE.stopIntake);

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(new InstantCommand(() -> TestFlywheel.INSTANCE.flywheelMotors.setPower(1)))
                .whenBecomesFalse(new InstantCommand(() -> TestFlywheel.INSTANCE.flywheelMotors.setPower(0)));
    }
}
