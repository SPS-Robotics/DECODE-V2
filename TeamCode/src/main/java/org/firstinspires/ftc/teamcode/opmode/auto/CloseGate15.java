package org.firstinspires.ftc.teamcode.opmode.auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "15 Artifact - Close Gate Intake")
public class CloseGate15 extends NextFTCOpMode {
    public CloseGate15() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Flywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private Pose startPose = new Pose(21, 125, Math.toRadians(324));
    private Pose scorePose = new Pose(50, 83);
    private Pose closeSpikePose = new Pose(23.2, 84);
    private Pose middleSpikePose = new Pose(18.8, 59.9);
    private Pose gateOpenPose = new Pose(15.7, 70);
    private Pose gateIntakePose = new Pose(10.5, 52.6);
    private Pose lastScorePose = new Pose(54.4, 124.6);

    public void initPose() {
        if (RobotState.ALLIANCE_COLOR == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
            scorePose = scorePose.mirror();
            closeSpikePose = closeSpikePose.mirror();
            middleSpikePose = middleSpikePose.mirror();
            gateOpenPose = gateOpenPose.mirror();
            gateIntakePose = gateIntakePose.mirror();
            lastScorePose = lastScorePose.mirror();
        }
    }

    double shootTimeSeconds = 2;
    double gateDelaySeconds = 2;

    public PathChain scorePreload, intakeMiddleSpike, scoreMiddleSpike, scoreToGate, gateIntake, gateToScore, intakeCloseSpike, scoreLastSpike;

    public void buildPaths() {
        scorePreload = follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(270))
                .build();
        intakeMiddleSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(53.2, 58.4), middleSpikePose))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();
        scoreMiddleSpike = follower().pathBuilder()
                .addPath(new BezierLine(middleSpikePose, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(220))
                .build();
        scoreToGate = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(33.5, 70.3), gateOpenPose))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))
                .build();
        gateIntake = follower().pathBuilder()
                .addPath(new BezierCurve(gateOpenPose, new Pose(16.7, 59.9), gateIntakePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();
        gateToScore = follower().pathBuilder()
                .addPath(new BezierLine(gateIntakePose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(220))
                .build();
        intakeCloseSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(46.7, 85.3), closeSpikePose))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))
                .build();
        scoreLastSpike = follower().pathBuilder()
                .addPath(new BezierLine(closeSpikePose, lastScorePose))
                .setConstantHeadingInterpolation(Math.toRadians(240))
                .build();
    }

    public Command shootArtifacts() {
        return new SequentialGroup(
                new ParallelDeadlineGroup(
                        new Delay(shootTimeSeconds),
                        Intake.INSTANCE.openGate,
                        Intake.INSTANCE.intakeArtifacts
                ),
                new ParallelGroup(
                        Intake.INSTANCE.closeGate,
                        Intake.INSTANCE.stopIntake
                )
        );
    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                // Drive to scorePose
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        Flywheel.INSTANCE.turnFlywheelOn
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
                new Delay(gateDelaySeconds),

                // Drive to scorePose
                Intake.INSTANCE.stopIntake,
                new FollowPath(gateToScore),

                shootArtifacts(),

                // Open gate
                new FollowPath(scoreToGate),

                // Gate intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(gateIntake),
                new Delay(gateDelaySeconds),

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
        initPose();
        buildPaths();
        follower().setStartingPose(startPose);
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Alliance Colour", RobotState.ALLIANCE_COLOR);
    }

    @Override
    public void onStartButtonPressed() {
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
