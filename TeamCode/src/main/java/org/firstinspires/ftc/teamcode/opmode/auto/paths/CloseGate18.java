package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import static org.firstinspires.ftc.teamcode.globals.Constants.Auto.SHOOT_TIME;
import static org.firstinspires.ftc.teamcode.globals.Constants.Auto.GATE_DELAY;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.LightingController;
import org.firstinspires.ftc.teamcode.util.MathUtils;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public abstract class CloseGate18 extends NextFTCOpMode {
    protected final RobotState.AllianceColor alliance;

    public CloseGate18(RobotState.AllianceColor alliance) {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Lift.INSTANCE, Intake.INSTANCE, Flywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );

        this.alliance = alliance;
    }

    private Pose startPose = new Pose(13.6, 111.4, Math.toRadians(0));

    private double preloadHeading = Math.toRadians(323);
    private Pose middleSpikePose = new Pose(12, 59.9, Math.toRadians(180));
    private Pose middleSpikeControl = new Pose(52.5, 56.5);

    private Pose scorePose = new Pose(55, 83);

    private double gateIntakeStartHeading = Math.toRadians(180);

    private Pose gateIntakePose = new Pose(12, 57.5, Math.toRadians(150));

    private Pose closeSpikePose = new Pose(16, 84, Math.toRadians(180));
    private Pose endPose = new Pose(24, 70, Math.toRadians(180));

    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
            preloadHeading = MathUtils.mirrorHeading(preloadHeading);
            middleSpikePose = middleSpikePose.mirror();
            middleSpikeControl = middleSpikeControl.mirror();
            scorePose = scorePose.mirror();
            gateIntakeStartHeading = MathUtils.mirrorHeading(gateIntakeStartHeading);
            gateIntakePose = gateIntakePose.mirror();
            closeSpikePose = closeSpikePose.mirror();
            endPose = endPose.mirror();
        }
    }

    private PathChain scorePreload, intakeMiddleSpike, scoreMiddleSpike, gateIntake, scoreGate, intakeCloseSpike, scoreCloseSpike, parkPath;

    private void buildPaths() {
        scorePreload = follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.3,
                                        HeadingInterpolator.linear(startPose.getHeading(), preloadHeading)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.3,
                                        1,
                                        HeadingInterpolator.tangent
                                )
                        )
                )
                .build();

        intakeMiddleSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, middleSpikeControl, middleSpikePose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.3,
                                        HeadingInterpolator.linear(preloadHeading, middleSpikePose.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.3,
                                        1,
                                        HeadingInterpolator.constant(middleSpikePose.getHeading())
                                )
                        )
                )
                .build();

        scoreMiddleSpike = follower().pathBuilder()
                .addPath(new BezierLine(middleSpikePose, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        gateIntake = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, gateIntakePose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.constant(gateIntakeStartHeading)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.linear(gateIntakeStartHeading, gateIntakePose.getHeading())
                                )
                        )
                )
                .build();

        scoreGate = follower().pathBuilder()
                .addPath(new BezierLine(gateIntakePose, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeCloseSpike = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, closeSpikePose))
                .setConstantHeadingInterpolation(closeSpikePose.getHeading())
                .build();

        scoreCloseSpike = follower().pathBuilder()
                .addPath(new BezierLine(closeSpikePose, scorePose))
                .setConstantHeadingInterpolation(closeSpikePose.getHeading())
                .build();

        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();

    }

    private Command shootArtifacts() {
        return new SequentialGroup(
                new SequentialGroup(
                        //Intake.INSTANCE.openGate,
                        Intake.INSTANCE.intakeArtifacts,
                        new Delay(SHOOT_TIME)
                ),
                new ParallelGroup(
                        Intake.INSTANCE.closeGate,
                        Intake.INSTANCE.stopIntake
                )
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                // Score Preload
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        Flywheel.INSTANCE.turnFlywheelOn,
                        Intake.INSTANCE.openGate,
                        new SequentialGroup(
                                new Delay(0.3),
                                Turret.INSTANCE.enableTracking
                        )
                ),

                shootArtifacts(),

                // Intake Middle Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeMiddleSpike),
                Intake.INSTANCE.stopIntake,

                // Score Middle Spike
                new ParallelGroup(
                        new FollowPath(scoreMiddleSpike),
                        Intake.INSTANCE.openGate
                        ),
                shootArtifacts(),

                // Gate Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(gateIntake),
                new Delay(GATE_DELAY),
                Intake.INSTANCE.stopIntake,

                // Score Gate
                new ParallelGroup(
                        new FollowPath(scoreGate),
                        Intake.INSTANCE.openGate
                ),
                shootArtifacts(),

                // Gate Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(gateIntake),
                new Delay(GATE_DELAY),
                Intake.INSTANCE.stopIntake,

                // Score Gate
                new ParallelGroup(
                        new FollowPath(scoreGate),
                        Intake.INSTANCE.openGate
                ),
                shootArtifacts(),

                // Intake Close Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeCloseSpike),
                Intake.INSTANCE.stopIntake,

                // Score Close Spike
                new ParallelGroup(
                        new FollowPath(scoreCloseSpike),
                        Intake.INSTANCE.openGate
                ),
                shootArtifacts(),

                // Gate Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(gateIntake),
                new Delay(GATE_DELAY),
                Intake.INSTANCE.stopIntake,

                // Score Gate
                new ParallelGroup(
                        new FollowPath(scoreGate),
                        Intake.INSTANCE.openGate
                ),
                shootArtifacts(),

                /*

                // Gate Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(gateIntake),
                new Delay(GATE_DELAY),
                Intake.INSTANCE.stopIntake,

                // Score Gate
                new ParallelGroup(
                        new FollowPath(scoreGate),
                        Intake.INSTANCE.openGate
                ),
                shootArtifacts(),

                 */

                // Park in front of gate
                new FollowPath(parkPath)
        );
    }

    @Override
    public void onInit() {
        RobotState.setAlliance(alliance);
        initPoses();
        buildPaths();
        follower().setStartingPose(startPose);
        LightingController.init();
        Turret.INSTANCE.setTurretPosition(0).schedule();
    }

    @Override
    public void onStartButtonPressed() {
        Lift.INSTANCE.disengageLift.schedule();
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        Pose robotPose = follower().getPose();
        if (robotPose.getX() != 0 && robotPose.getY() != 0 && robotPose.getHeading() != 0) {
            RobotState.AUTO_END_POSE = robotPose;
            RobotState.AUTO_END_X = robotPose.getX();
            RobotState.AUTO_END_Y = robotPose.getY();
            RobotState.AUTO_END_HEADING = robotPose.getHeading();
        }
        RobotState.TURRET_END_POS = Turret.INSTANCE.getTurretPosition();
        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());
        telemetry.addData("Alliance", RobotState.ALLIANCE_COLOR);
        telemetry.addData("Goal Pose", RobotState.GOAL_POSE);
        telemetry.update();
        LightingController.get().update();
    }

    @Override
    public void onStop() {
        Flywheel.INSTANCE.turnFlywheelOff.schedule();
        Turret.INSTANCE.disableTracking.schedule();
    }
}