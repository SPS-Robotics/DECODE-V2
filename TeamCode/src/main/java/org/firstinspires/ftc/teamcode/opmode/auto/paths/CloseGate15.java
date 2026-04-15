package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import static org.firstinspires.ftc.teamcode.globals.Constants.Auto.SHOOT_TIME;

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
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public abstract class CloseGate15 extends NextFTCOpMode {
    protected final RobotState.AllianceColor alliance;

    public CloseGate15(RobotState.AllianceColor alliance) {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Lift.INSTANCE, Intake.INSTANCE, Flywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );

        this.alliance = alliance;
    }

    private Pose startPose = new Pose(14.0, 112.093, Math.toRadians(270));
    private Pose preloadScorePose = new Pose(48, 96.3, Math.toRadians(270));

    private Pose middleSpikePose = new Pose(24, 59.9, Math.toRadians(180));
    private Pose middleSpikeControl = new Pose(64, 59.9);

    private Pose scorePose = new Pose(57.5, 86.7);

    private double gateIntakeStartHeading = Math.toRadians(225);

    private Pose gateIntakePose = new Pose(11, 58.4, Math.toRadians(156));
    private Pose gateIntakeControl = new Pose (48, 59);

    private Pose gateScoreControl = new Pose(21, 50.5);

    private Pose closeSpikePose = new Pose(24, 84);
    private Pose lastScorePose = new Pose(52, 111.4);

    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
            preloadScorePose = preloadScorePose.mirror();
            middleSpikePose = middleSpikePose.mirror();
            middleSpikeControl = middleSpikeControl.mirror();
            scorePose = scorePose.mirror();
            gateIntakeStartHeading = MathUtils.mirrorHeading(gateIntakeStartHeading);
            gateIntakePose = gateIntakePose.mirror();
            gateIntakeControl = gateIntakeControl.mirror();
            gateScoreControl = gateScoreControl.mirror();
            closeSpikePose = closeSpikePose.mirror();
            lastScorePose = lastScorePose.mirror();
        }
    }

    private PathChain scorePreload, intakeMiddleSpike, scoreMiddleSpike, gateIntake, scoreGate, intakeCloseSpike, scoreLastSpike;
    private void buildPaths() {
        scorePreload = follower().pathBuilder()
                .addPath(new BezierLine(startPose, preloadScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                .build();

        intakeMiddleSpike = follower().pathBuilder()
                .addPath(new BezierCurve(preloadScorePose, middleSpikeControl, middleSpikePose))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), middleSpikePose.getHeading())
                .build();

        scoreMiddleSpike = follower().pathBuilder()
                .addPath(new BezierLine(middleSpikePose, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        gateIntake = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, gateIntakeControl, gateIntakePose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.linear(gateIntakeStartHeading, gateIntakePose.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.constant(gateIntakePose.getHeading())
                                )
                        )
                )
                .build();

        scoreGate = follower().pathBuilder()
                .addPath(new BezierCurve(gateIntakePose, gateScoreControl, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeCloseSpike = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, closeSpikePose))
                .setTangentHeadingInterpolation()
                .build();

        scoreLastSpike = follower().pathBuilder()
                .addPath(new BezierLine(closeSpikePose, lastScorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
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
                new Delay(1.2),
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
                new Delay(1.2),
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
                        new FollowPath(scoreLastSpike),
                        Intake.INSTANCE.openGate
                ),
                shootArtifacts()
        );
    }

    @Override
    public void onInit() {
        RobotState.setAlliance(alliance);
        initPoses();
        buildPaths();
        follower().setStartingPose(startPose);
        LightingController.init();
    }

    @Override
    public void onStartButtonPressed() {
        Lift.INSTANCE.disengageLift.schedule();
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        Pose robotPose = follower().getPose();
        RobotState.AUTO_END_POSE = robotPose;
        RobotState.AUTO_END_X = robotPose.getX();
        RobotState.AUTO_END_Y = robotPose.getY();
        RobotState.AUTO_END_HEADING = robotPose.getHeading();
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