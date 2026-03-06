package org.firstinspires.ftc.teamcode.opmode.auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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

@Autonomous(name="12 Artifact - ALL 3 SPIKES")
public class BackupAuto9 extends NextFTCOpMode {
    public BackupAuto9() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Lift.INSTANCE, Intake.INSTANCE, Flywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private Pose startPose = new Pose(17.5, 120, Math.toRadians(324));
    private Pose scorePose = new Pose(41.2, 96.3);
    private Pose closeSpikePose = new Pose(20, 84);
    private Pose middleSpikePose = new Pose(18.8, 59.9);
    private Pose gateOpenPose = new Pose(22.7, 70);
    private Pose farSpikePose = new Pose(20.7, 35);
    private Pose lastScorePose = new Pose(54.4, 124.6);

    public void initPose() {
        if (RobotState.ALLIANCE_COLOR == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
            scorePose = scorePose.mirror();
            closeSpikePose = closeSpikePose.mirror();
            middleSpikePose = middleSpikePose.mirror();
            gateOpenPose = gateOpenPose.mirror();
            farSpikePose = farSpikePose.mirror();
            lastScorePose = lastScorePose.mirror();
        }
    }

    double shootTimeSeconds = 2;

    public PathChain scorePreload, intakeCloseSpike, scoreCloseSpike, intakeMiddleSpike, scoreMiddleSpike;

    public void buildPaths() {
        scorePreload = follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180))
                .build();
        intakeCloseSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(57.8, 82), closeSpikePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scoreCloseSpike = follower().pathBuilder()
                .addPath(new BezierLine(closeSpikePose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(240))
                .build();
        intakeMiddleSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(53.2, 58.4), middleSpikePose))
                .setLinearHeadingInterpolation(Math.toRadians(288), Math.toRadians(180))
                .build();
        scoreMiddleSpike = follower().pathBuilder()
                .addPath(new BezierLine(middleSpikePose, lastScorePose))
                .setConstantHeadingInterpolation(Math.toRadians(245))
                .build();
    }

    public Command shootArtifacts() {
        return new SequentialGroup(
                new SequentialGroup(
                        Flywheel.INSTANCE.turnFlywheelOn,
                        new Delay(1.8)
                ),
                new SequentialGroup(
                        Intake.INSTANCE.openGate,
                        Intake.INSTANCE.intakeArtifacts,
                        new Delay(shootTimeSeconds)
                ),
                new ParallelGroup(
                        Intake.INSTANCE.closeGate,
                        Intake.INSTANCE.stopIntake,
                        Flywheel.INSTANCE.turnFlywheelOff
                )
        );
    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        new SequentialGroup(
                                new Delay(0.8),
                                Turret.INSTANCE.enableTracking
                )),

                shootArtifacts(),

                // Intake Close Spike + Open Gate
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeCloseSpike),
                Intake.INSTANCE.stopIntake,

                // Drive to scorePose
                new FollowPath(scoreCloseSpike),
                shootArtifacts(),

                // Intake Middle Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeMiddleSpike),

                // Drive to scorePose
                Intake.INSTANCE.stopIntake,
                new FollowPath(scoreMiddleSpike),

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
        Lift.INSTANCE.disengageLift.schedule();
        Turret.INSTANCE.setTurretPosition(0).schedule();
        Lift.INSTANCE.disengageLift.schedule();
        //Turret.INSTANCE.enableTracking.schedule();
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
        telemetry.addData("Distance", robotPose.distanceFrom(RobotState.GOAL_POSE));
        telemetry.update();
    }

    @Override
    public void onStop() {
        Flywheel.INSTANCE.turnFlywheelOff.schedule();
        Turret.INSTANCE.disableTracking.schedule();
    }
}


