package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import static org.firstinspires.ftc.teamcode.globals.Constants.Auto.GATE_DELAY;
import static org.firstinspires.ftc.teamcode.globals.Constants.Auto.SHOOT_TIME;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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

    private Pose startPose = new Pose(17.5, 120, Math.toRadians(324));

    private Pose scorePose = new Pose(50, 83, Math.toRadians(260));

    private Pose closeSpikePose = new Pose(23.2, 84, Math.toRadians(180));
    private Pose closeSpikeControl = new Pose(46.7, 85.3);

    private Pose middleSpikePose = new Pose(18.8, 59.9, Math.toRadians(180));
    private Pose middleSpikeControl = new Pose(53.2, 58.4);

    private double middleScoreHeading = Math.toRadians(220);

    private Pose gateOpenPose = new Pose(20.7, 70, Math.toRadians(180));
    private Pose gateOpenControl = new Pose(33.5, 70.3);

    private Pose gateIntakePose = new Pose(12.75, 42.6, Math.toRadians(100));
    private Pose gateIntakeControl = new Pose(16.7, 59.9);

    private Pose lastScorePose = new Pose(54.4, 124.6, Math.toRadians(260));

    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
            scorePose = scorePose.mirror();
            closeSpikePose = closeSpikePose.mirror();
            closeSpikeControl = closeSpikeControl.mirror();
            middleSpikePose = middleSpikePose.mirror();
            middleSpikeControl = middleSpikeControl.mirror();
            middleScoreHeading = MathUtils.mirrorHeading(middleScoreHeading);
            gateOpenPose = gateOpenPose.mirror();
            gateOpenControl = gateOpenControl.mirror();
            gateIntakePose = gateIntakePose.mirror();
            gateIntakeControl = gateIntakeControl.mirror();
            lastScorePose = lastScorePose.mirror();
        }
    }

    private PathChain scorePreload, intakeMiddleSpike, scoreMiddleSpike, scoreToGate, gateIntake, gateToScore, intakeCloseSpike, scoreLastSpike;

    private void buildPaths() {
        scorePreload = follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        intakeMiddleSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, middleSpikeControl, middleSpikePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), middleSpikePose.getHeading())
                .build();
        scoreMiddleSpike = follower().pathBuilder()
                .addPath(new BezierLine(middleSpikePose, scorePose))
                .setConstantHeadingInterpolation(middleScoreHeading)
                .build();
        scoreToGate = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, gateOpenControl, gateOpenPose))
                .setLinearHeadingInterpolation(middleScoreHeading, gateOpenPose.getHeading())
                .build();
        gateIntake = follower().pathBuilder()
                .addPath(new BezierCurve(gateOpenPose, gateIntakeControl, gateIntakePose))
                .setLinearHeadingInterpolation(gateOpenPose.getHeading(), gateIntakePose.getHeading())
                .build();
        gateToScore = follower().pathBuilder()
                .addPath(new BezierLine(gateIntakePose, scorePose))
                .setLinearHeadingInterpolation(gateIntakePose.getHeading(), scorePose.getHeading())
                .build();
        intakeCloseSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, closeSpikeControl, closeSpikePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), closeSpikePose.getHeading())
                .build();
        scoreLastSpike = follower().pathBuilder()
                .addPath(new BezierLine(closeSpikePose, lastScorePose))
                .setConstantHeadingInterpolation(lastScorePose.getHeading())
                .build();
    }

    private Command shootArtifacts() {
        return new SequentialGroup(
                new SequentialGroup(
                        Intake.INSTANCE.openGate,
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
                // Drive to scorePose
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        Flywheel.INSTANCE.turnFlywheelOn,
                        Turret.INSTANCE.enableTracking
                ),

                shootArtifacts(),
                // Intake Middle Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeMiddleSpike),

                // Drive to scorePose
                Intake.INSTANCE.stopIntake,
                new FollowPath(scoreMiddleSpike),

                shootArtifacts(),

                // Open gate
                new FollowPath(scoreToGate),

                // Gate intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(gateIntake),
                new ParallelRaceGroup(
                        new WaitUntil(() -> Intake.INSTANCE.hasThreeBalls),
                        new Delay(GATE_DELAY)
                ),

                // Drive to scorePose
                Intake.INSTANCE.stopIntake,
                new FollowPath(gateToScore),

                shootArtifacts(),

                // Open gate
                new FollowPath(scoreToGate),

                // Gate intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(gateIntake),
                new ParallelRaceGroup(
                        new WaitUntil(() -> Intake.INSTANCE.hasThreeBalls),
                        new Delay(GATE_DELAY)
                ),

                // Drive to scorePose
                Intake.INSTANCE.stopIntake,
                new FollowPath(gateToScore),

                shootArtifacts(),

                // Intake Close Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeCloseSpike),

                // Drive to lastPose (off of LAUNCH line)
                Intake.INSTANCE.stopIntake,
                new FollowPath(scoreLastSpike),

                shootArtifacts()
        );
    }

    @Override
    public void onInit() {
        RobotState.setAlliance(alliance);
        initPoses();
        buildPaths();
        follower().setStartingPose(startPose);
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Alliance Colour", RobotState.ALLIANCE_COLOR);
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
        RobotState.TURRET_END_POS = Turret.INSTANCE.getTurretPosition();
        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());
        telemetry.addData("Alliance", RobotState.ALLIANCE_COLOR);
        telemetry.addData("Goal Pose", RobotState.GOAL_POSE);
        telemetry.update();
    }

    @Override
    public void onStop() {
        Flywheel.INSTANCE.turnFlywheelOff.schedule();
        Turret.INSTANCE.disableTracking.schedule();
    }
}
