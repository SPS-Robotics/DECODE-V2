package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.TuningFlywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
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

@TeleOp(name="Flywheel Tuner")
public class FlywheelTuning extends NextFTCOpMode {
    public FlywheelTuning() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, TuningFlywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private final MotorEx frontLeft = new MotorEx("frontLeft").brakeMode().zeroed().reversed();
    private final MotorEx frontRight = new MotorEx("frontRight").brakeMode().zeroed();
    private final MotorEx backLeft = new MotorEx("backLeft").brakeMode().zeroed().reversed();
    private final MotorEx backRight = new MotorEx("backRight").brakeMode().zeroed();

    private final IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    private final double[] hoodIncrements = {0.05, 0.01, 0.001};
    private final double[] flywheelIncrements = {50, 10, 1};

    private int hoodIncrementIndex = 0;
    private int flywheelIncrementIndex = 0;

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

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(TuningFlywheel.INSTANCE.moveHoodByValue(hoodIncrements[hoodIncrementIndex]));
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(TuningFlywheel.INSTANCE.moveHoodByValue(-hoodIncrements[hoodIncrementIndex]));


        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(TuningFlywheel.INSTANCE.moveFlywheelByValue(flywheelIncrements[flywheelIncrementIndex]));
        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(TuningFlywheel.INSTANCE.moveFlywheelByValue(-flywheelIncrements[flywheelIncrementIndex]));

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(
                        new InstantCommand(() -> {
                                hoodIncrementIndex = (hoodIncrementIndex + 1) % hoodIncrements.length;
                                flywheelIncrementIndex = (flywheelIncrementIndex + 1) % flywheelIncrements.length;
                        }));

        Gamepads.gamepad1().triangle()
                .whenBecomesTrue(Turret.INSTANCE.toggleTracking);
    }
}
