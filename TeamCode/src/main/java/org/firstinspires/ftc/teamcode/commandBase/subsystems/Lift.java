package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.globals.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Lift implements Subsystem {
    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    private final MotorEx backLeft = new MotorEx("backLeft").brakeMode();
    private final MotorEx backRight = new MotorEx("backRight").brakeMode();
    private final MotorEx frontLeft = new MotorEx("frontLeft").brakeMode().reversed();
    private final MotorEx frontRight = new MotorEx("frontRight").brakeMode().reversed();
    private final ServoEx leftServo = new ServoEx("leftServo");
    private final ServoEx rightServo = new ServoEx("rightServo");

    private enum LiftState {
        DISENGAGED, ENGAGING, LIFTING, LIFTED
    };

    LiftState currentLift = LiftState.DISENGAGED;

    public Command engageLift = new ParallelGroup(
            new SetPosition(leftServo, Constants.Lift.LEFT_SERVO_ENGAGED),
            new SetPosition(rightServo, Constants.Lift.RIGHT_SERVO_ENGAGED)
    );

    public Command disengageLift = new ParallelGroup(
            new SetPosition(leftServo, Constants.Lift.LEFT_SERVO_DISENGAGED),
            new SetPosition(rightServo, Constants.Lift.RIGHT_SERVO_DISENGAGED)
    );

    public Command startLift = new SequentialGroup(
            new InstantCommand(() -> currentLift = LiftState.ENGAGING),
            new Delay(Constants.Lift.ENGAGE_TIME),
            new InstantCommand(() -> currentLift = LiftState.LIFTING)
    );
    public Command stopLift = new InstantCommand(() -> currentLift = LiftState.LIFTED);

    @Override
    public void periodic() {
        if (currentLift == LiftState.ENGAGING) {
            double power = Constants.Lift.ENGAGING_POWER;
            backLeft.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(-power);
            frontRight.setPower(-power);
        }

        if (currentLift == LiftState.LIFTING) {
            double power = Constants.Lift.LIFT_POWER;
            backLeft.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        if (currentLift == LiftState.LIFTED) {
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
    }
}
