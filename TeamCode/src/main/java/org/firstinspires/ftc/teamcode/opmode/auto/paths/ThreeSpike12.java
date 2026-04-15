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

public abstract class ThreeSpike12 extends NextFTCOpMode {
    protected final RobotState.AllianceColor alliance;

    public ThreeSpike12(RobotState.AllianceColor alliance) {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Lift.INSTANCE, Intake.INSTANCE, Flywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );

        this.alliance = alliance;
    }

    private Pose startPose = new Pose(17.5, 120, Math.toRadians(324));
    private Pose scorePose = new Pose(41.2, 96.3, Math.toRadians(260));

    private double closeStartHeading = Math.toRadians(220);
    private Pose closeSpikePose = new Pose(21, 84, Math.toRadians(180));
    private Pose closeSpikeControl = new Pose(67.8, 82);

    private double middleStartHeading = Math.toRadians(288);
    private Pose middleSpikePose = new Pose(20.8, 57.9, Math.toRadians(180));
    private Pose middleSpikeControl = new Pose(53.2, 58.4);
    private double middleScoreHeading = Math.toRadians(240);

    private Pose gateOpenPose = new Pose(24.5, 70, Math.toRadians(180));
    private Pose gateOpenControl = new Pose(36.1, 75.5);

    private double farStartHeading = Math.toRadians(280);
    private Pose farSpikePose = new Pose(20.7, 35, Math.toRadians(180));
    private Pose farSpikeControl = new Pose(52.5, 31);

    private double lastStartHeading = Math.toRadians(250);
    private Pose lastScorePose = new Pose(54.4, 124.6, Math.toRadians(280));

    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
            scorePose = scorePose.mirror();

            closeStartHeading = MathUtils.mirrorHeading(closeStartHeading);
            closeSpikePose = closeSpikePose.mirror();
            closeSpikeControl = closeSpikeControl.mirror();

            middleStartHeading = MathUtils.mirrorHeading(middleStartHeading);
            middleSpikePose = middleSpikePose.mirror();
            middleSpikeControl = middleSpikeControl.mirror();
            middleScoreHeading = MathUtils.mirrorHeading(middleScoreHeading);

            gateOpenPose = gateOpenPose.mirror();
            gateOpenControl = gateOpenControl.mirror();

            farStartHeading = MathUtils.mirrorHeading(farStartHeading);
            farSpikePose = farSpikePose.mirror();
            farSpikeControl = farSpikeControl.mirror();

            lastStartHeading = MathUtils.mirrorHeading(lastStartHeading);
            lastScorePose = lastScorePose.mirror();
        }
    }

    private PathChain scorePreload, intakeCloseSpike, openGate, scoreCloseSpike, intakeMiddleSpike, scoreMiddleSpike, intakeFarSpike, scoreLastSpike;

    private void buildPaths() {
        scorePreload = follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        intakeCloseSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, closeSpikeControl, closeSpikePose))
                .setLinearHeadingInterpolation(closeStartHeading, closeSpikePose.getHeading())
                .build();
        openGate = follower().pathBuilder()
                .addPath(new BezierCurve(closeSpikePose, gateOpenControl, gateOpenPose))
                .setConstantHeadingInterpolation(gateOpenPose.getHeading())
                .build();
        scoreCloseSpike = follower().pathBuilder()
                .addPath(new BezierLine(gateOpenPose, scorePose))
                .setLinearHeadingInterpolation(gateOpenPose.getHeading(), scorePose.getHeading())
                .build();
        intakeMiddleSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, middleSpikeControl, middleSpikePose))
                .setLinearHeadingInterpolation(middleStartHeading, middleSpikePose.getHeading())
                .build();
        scoreMiddleSpike = follower().pathBuilder()
                .addPath(new BezierLine(middleSpikePose, scorePose))
                .setLinearHeadingInterpolation(middleScoreHeading, scorePose.getHeading())
                .build();
        intakeFarSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, farSpikeControl, farSpikePose))
                .setLinearHeadingInterpolation(farStartHeading, farSpikePose.getHeading())
                .build();
        scoreLastSpike = follower().pathBuilder()
                .addPath(new BezierLine(farSpikePose, lastScorePose))
                .setLinearHeadingInterpolation(lastStartHeading, lastScorePose.getHeading())
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
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        Flywheel.INSTANCE.turnFlywheelOn,
                        new SequentialGroup(
                                new Delay(0.3),
                                Turret.INSTANCE.enableTracking
                        )
                ),

                shootArtifacts(),

                // Intake Close Spike + Open Gate
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeCloseSpike),
                Intake.INSTANCE.stopIntake,
                new FollowPath(openGate),

                // Drive to scorePose
                new FollowPath(scoreCloseSpike),
                shootArtifacts(),

                // Intake Middle Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeMiddleSpike),

                // Drive to scorePose
                Intake.INSTANCE.stopIntake,
                new FollowPath(scoreMiddleSpike),

                shootArtifacts(),

                // Intake Far Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeFarSpike),

                // Drive to lastScorePose
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
        LightingController.init();
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Alliance Colour", RobotState.ALLIANCE_COLOR);
        LightingController.get().update();
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
