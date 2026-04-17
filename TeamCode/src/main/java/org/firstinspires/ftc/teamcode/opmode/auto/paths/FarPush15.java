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
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public abstract class FarPush15 extends NextFTCOpMode {
    protected final RobotState.AllianceColor alliance;

    public FarPush15(RobotState.AllianceColor alliance) {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Lift.INSTANCE, Intake.INSTANCE, Flywheel.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );

        this.alliance = alliance;
    }

    private Pose startPose = new Pose(64.9, 8, Math.toRadians(180));
    private Pose pushToPose = new Pose(50, 8);
    private Pose middleSpikePose = new Pose(20, 58, Math.toRadians(180));
    private Pose middleSpikeControl = new Pose(60, 60);

    private Pose gatePose = new Pose(18, 70, Math.toRadians(180));
    private Pose gateControl = new Pose(27, 65);
    private Pose scorePose = new Pose(57.5, 86.7);

    private double gateIntakeStartHeading = Math.toRadians(225);

    private Pose gateIntakePose = new Pose(11, 58.4, Math.toRadians(156));
    private Pose gateIntakeControl = new Pose (48, 59);

    private Pose gateScoreControl = new Pose(21, 50.5);

    private double farStartHeading = Math.toRadians(225);
    private Pose farSpikePose = new Pose(24, 36, Math.toRadians(180));
    private Pose farSpikeControl = new Pose(57.5, 36);
    private Pose closeSpikePose = new Pose(24, 84);
    private Pose lastScorePose = new Pose(51, 113);

    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
            pushToPose = pushToPose.mirror();
            middleSpikePose = middleSpikePose.mirror();
            middleSpikeControl = middleSpikeControl.mirror();

            gatePose = gatePose.mirror();
            gateControl = gateControl.mirror();

            scorePose = scorePose.mirror();
            gateIntakeStartHeading = MathUtils.mirrorHeading(gateIntakeStartHeading);
            gateIntakePose = gateIntakePose.mirror();
            gateIntakeControl = gateIntakeControl.mirror();
            gateScoreControl = gateScoreControl.mirror();

            farStartHeading = MathUtils.mirrorHeading(farStartHeading);
            farSpikePose = farSpikePose.mirror();
            farSpikeControl = farSpikeControl.mirror();
            closeSpikePose = closeSpikePose.mirror();
            lastScorePose = lastScorePose.mirror();
        }
    }

    private PathChain pushAlliance, intakeMiddleSpike, openGate, scoreMiddleSpike, gateIntake, scoreGate, intakeFarSpike, scoreFarSpike,  intakeCloseSpike, scoreLastSpike;
    private void buildPaths() {
        pushAlliance = follower().pathBuilder()
                .addPath(new BezierLine(startPose, pushToPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        intakeMiddleSpike = follower().pathBuilder()
                .addPath(new BezierCurve(pushToPose, middleSpikeControl, middleSpikePose))
                .setConstantHeadingInterpolation(middleSpikePose.getHeading())
                .build();

        openGate = follower().pathBuilder()
                .addPath(new BezierCurve(middleSpikePose, gateControl, gatePose))
                .setConstantHeadingInterpolation(middleSpikePose.getHeading())
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

        intakeFarSpike = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, farSpikeControl, farSpikePose))
                .setLinearHeadingInterpolation(farStartHeading, farSpikePose.getHeading())
                .build();

        scoreFarSpike = follower().pathBuilder()
                .addPath(new BezierLine(farSpikePose, scorePose))
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
                        //Flywheel.INSTANCE.turnFlywheelOn,
                        Intake.INSTANCE.openGate,
                        //new WaitUntil(() -> Flywheel.INSTANCE.atSpeed),
                        new SequentialGroup(
                                new Delay(0.3),
                                Turret.INSTANCE.enableTracking
                        )
                ),

                shootArtifacts(),

                // Push alliance out of launch zone
                new FollowPath(pushAlliance),

                // Intake Middle Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeMiddleSpike),
                Intake.INSTANCE.stopIntake,

                new FollowPath(openGate),

                // Score Middle Spike
                new ParallelGroup(
                        Intake.INSTANCE.openGate,
                        new FollowPath(scoreMiddleSpike)
                ),
                shootArtifacts(),

                // Gate Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(gateIntake),
                new Delay(1.2),
                Intake.INSTANCE.stopIntake,

                // Score Gate
                new ParallelGroup(
                        Intake.INSTANCE.openGate,
                        new FollowPath(scoreGate)
                ),
                shootArtifacts(),

                // Intake Far Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeFarSpike),
                Intake.INSTANCE.stopIntake,

                // Score Far Spike
                new ParallelGroup(
                        Intake.INSTANCE.openGate,
                        new FollowPath(scoreFarSpike)
                ),
                shootArtifacts(),

                // Intake Close Spike
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(intakeCloseSpike),
                Intake.INSTANCE.stopIntake,

                // Score Close Spike
                new ParallelGroup(
                        Intake.INSTANCE.openGate,
                        new FollowPath(scoreLastSpike)
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
