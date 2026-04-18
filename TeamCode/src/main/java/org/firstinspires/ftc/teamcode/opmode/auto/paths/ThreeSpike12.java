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

    private Pose startPose = new Pose(14, 112.093, Math.toRadians(270));
    private Pose preloadScorePose = new Pose(48, 96.3, Math.toRadians(270));

    private Pose closeSpikePose = new Pose(16, 84, Math.toRadians(180));
    private Pose closeSpikeControl = new Pose(58, 84);

    private Pose gateOpenPose = new Pose(18, 70, Math.toRadians(180));
    private Pose gateOpenControl = new Pose(30, 77);

    private Pose scorePose = new Pose(58, 80);

    private Pose middleSpikePose = new Pose(20, 60, Math.toRadians(180));
    private Pose middleSpikeControl = new Pose(65, 60);

    private Pose farSpikePose = new Pose(20, 36);
    private Pose farSpikeControl = new Pose(70, 36);

    private Pose lastScorePose = new Pose(52, 111.4);

    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
            scorePose = scorePose.mirror();
            preloadScorePose = preloadScorePose.mirror();

            closeSpikePose = closeSpikePose.mirror();
            closeSpikeControl = closeSpikeControl.mirror();

            middleSpikePose = middleSpikePose.mirror();
            middleSpikeControl = middleSpikeControl.mirror();

            gateOpenPose = gateOpenPose.mirror();
            gateOpenControl = gateOpenControl.mirror();

            farSpikePose = farSpikePose.mirror();
            farSpikeControl = farSpikeControl.mirror();

            lastScorePose = lastScorePose.mirror();
        }
    }

    private PathChain scorePreload, intakeCloseSpike, openGate, scoreCloseSpike, intakeMiddleSpike, scoreMiddleSpike, intakeFarSpike, scoreLastSpike;

    private void buildPaths() {
        scorePreload = follower().pathBuilder()
                .addPath(new BezierLine(startPose, preloadScorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        intakeCloseSpike = follower().pathBuilder()
                .addPath(new BezierCurve(preloadScorePose, closeSpikeControl, closeSpikePose))
                .setConstantHeadingInterpolation(closeSpikePose.getHeading())
                .build();
        openGate = follower().pathBuilder()
                .addPath(new BezierCurve(closeSpikePose, gateOpenControl, gateOpenPose))
                .setConstantHeadingInterpolation(gateOpenPose.getHeading())
                .build();
        scoreCloseSpike = follower().pathBuilder()
                .addPath(new BezierLine(gateOpenPose, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intakeMiddleSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, middleSpikeControl, middleSpikePose))
                .setConstantHeadingInterpolation(middleSpikePose.getHeading())
                .build();
        scoreMiddleSpike = follower().pathBuilder()
                .addPath(new BezierLine(middleSpikePose, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intakeFarSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, farSpikeControl, farSpikePose))
                .setTangentHeadingInterpolation()
                .build();
        scoreLastSpike = follower().pathBuilder()
                .addPath(new BezierLine(farSpikePose, lastScorePose))
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

                // Intake Close Spike + Open Gate
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeCloseSpike),
                Intake.INSTANCE.stopIntake,
                new FollowPath(openGate),

                // Drive to scorePose
                new ParallelGroup(
                        new FollowPath(scoreCloseSpike),
                        Intake.INSTANCE.openGate
                ),
                shootArtifacts(),

                // Intake Middle Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeMiddleSpike),
                Intake.INSTANCE.stopIntake,

                // Drive to scorePose
                new ParallelGroup(
                        new FollowPath(scoreMiddleSpike),
                        Intake.INSTANCE.openGate
                ),
                shootArtifacts(),

                // Intake Far Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeFarSpike),
                Intake.INSTANCE.stopIntake,

                // Drive to lastScorePose
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
