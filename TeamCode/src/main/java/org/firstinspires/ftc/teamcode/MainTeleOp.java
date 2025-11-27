package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
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
                new SubsystemComponent(Intake.INSTANCE)
                //new PedroComponent(Constants::createFollower)
        );
    }

    // Pose goalPose, loadingZone;

    private final MotorEx frontLeft = new MotorEx("frontLeft").brakeMode().zeroed().reversed();
    private final MotorEx frontRight = new MotorEx("frontRight").brakeMode().zeroed();
    private final MotorEx backLeft = new MotorEx("backLeft").brakeMode().zeroed().reversed();
    private final MotorEx backRight = new MotorEx("backRight").brakeMode().zeroed();

    private final IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    @Override
    public void onInit() {
        /*
        PedroComponent.follower().setPose(RobotState.autoEndPose);
        RobotState.Alliance alliance = RobotState.alliance;

        if (alliance == RobotState.Alliance.BLUE) {
            goalPose = SoftwareConstants.Poses.blueGoalPose;
            loadingZone = SoftwareConstants.Poses.blueLoadingZoneCorner;
        }
        else {
            goalPose = SoftwareConstants.Poses.redGoalPose;
            loadingZone = SoftwareConstants.Poses.redLoadingZoneCorner;
        }
         */
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

        /*
        Gamepads.gamepad1().rightBumper().and(Gamepads.gamepad1().rightTrigger().greaterThan(0.05))
                .whenBecomesTrue(new SequentialGroup(
                        Flywheel.INSTANCE.turnFlywheelOn,
                        Intake.INSTANCE.openGate
                ))
                .whenBecomesFalse(new ParallelGroup(
                        Flywheel.INSTANCE.turnFlywheelOff,
                        Intake.INSTANCE.closeGate
                ));

        Gamepads.gamepad2().cross()
                .whenBecomesTrue(new InstantCommand(() -> PedroComponent.follower().setPose(loadingZone)));

         */
    }

    @Override
    public void onUpdate() {
    }
    @Override
    public void onStop() { }
}
