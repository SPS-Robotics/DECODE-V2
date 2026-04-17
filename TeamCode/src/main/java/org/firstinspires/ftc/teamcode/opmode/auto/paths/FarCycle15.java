package org.firstinspires.ftc.teamcode.opmode.auto.paths;

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

public class FarCycle15 extends NextFTCOpMode {

    protected final RobotState.AllianceColor alliance;

    public FarCycle15(RobotState.AllianceColor alliance) {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Lift.INSTANCE, Intake.INSTANCE, Flywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );

        this.alliance = alliance;
    }

    private double constantHeading = Math.toRadians(180);
    private final double intakeX = 12;

    private Pose startPose = new Pose(55.093, 8, constantHeading);
    private Pose cornerIntakePose = new Pose(intakeX, 9);
    private Pose scorePose = new Pose(48, 9);
    private Pose farSpikePose = new Pose(20.8, 36);
    private Pose farSpikeControl = new Pose(56, 36);
    private Pose sweepStartPose = new Pose(intakeX, 30);
    private Pose sweepStartControl = new Pose(48, 30);
    private Pose sweepIntakeControl = new Pose(18, 19);
    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            constantHeading = MathUtils.mirrorHeading(constantHeading);
            startPose = startPose.mirror();
            cornerIntakePose = cornerIntakePose.mirror();
            scorePose = scorePose.mirror();
            farSpikePose = farSpikePose.mirror();
            farSpikeControl = farSpikeControl.mirror();
            sweepStartPose = sweepStartPose.mirror();
            sweepStartControl = sweepStartControl.mirror();
            sweepIntakeControl = sweepIntakeControl.mirror();
        }
    }

    private PathChain startIntake, intakeCorner, scoreCorner, intakeFarSpike, scoreFarSpike, sweepStart, sweepIntake;

    private void buildPaths() {
        startIntake = follower().pathBuilder()
                .addPath(new BezierLine(startPose, cornerIntakePose))
                .setConstantHeadingInterpolation(constantHeading)
                .build();
        intakeCorner = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, cornerIntakePose))
                .setConstantHeadingInterpolation(constantHeading)
                .build();
        scoreCorner = follower().pathBuilder()
                .addPath(new BezierLine(cornerIntakePose, scorePose))
                .setConstantHeadingInterpolation(constantHeading)
                .build();
        intakeFarSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, farSpikeControl, farSpikePose))
                .setConstantHeadingInterpolation(constantHeading)
                .build();
        scoreFarSpike = follower().pathBuilder()
                .addPath(new BezierLine(farSpikePose, scorePose))
                .setConstantHeadingInterpolation(constantHeading)
                .build();
        sweepStart = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, sweepStartControl, sweepStartPose))
                .setConstantHeadingInterpolation(constantHeading)
                .build();
        sweepIntake = follower().pathBuilder()
                .addPath(new BezierCurve(sweepStartPose, sweepIntakeControl, cornerIntakePose))
                .setConstantHeadingInterpolation(constantHeading)
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
                // Score Preload
                new ParallelGroup(
                        Flywheel.INSTANCE.turnFlywheelOn,
                        new SequentialGroup(
                                new Delay(0.3),
                                Turret.INSTANCE.enableTracking
                        )
                ),

                shootArtifacts(),

                // Corner Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(startIntake),
                Intake.INSTANCE.stopIntake,

                // Score Corner Intake
                new FollowPath(scoreCorner),
                shootArtifacts(),

                // Intake Far Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeFarSpike),
                Intake.INSTANCE.stopIntake,

                // Score Far Spike
                new FollowPath(scoreFarSpike),
                shootArtifacts(),

                // Sweep Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(sweepStart),
                new FollowPath(sweepIntake),
                Intake.INSTANCE.stopIntake,

                // Score Sweep
                new FollowPath(scoreCorner),
                shootArtifacts(),

                // Corner Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeCorner),
                Intake.INSTANCE.stopIntake,

                // Score Corner Intake
                new FollowPath(scoreCorner),
                shootArtifacts(),

                // Leave
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeCorner),
                Intake.INSTANCE.stopIntake
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
