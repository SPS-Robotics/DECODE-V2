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
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
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
    private Pose middleSpikePose = new Pose(13, 57.5, Math.toRadians(180));
    private Pose middleSpikeControl = new Pose(52.5, 56.5);

    private Pose scoreMiddleSpikeControl = new Pose(50, 70);

    private Pose scorePose = new Pose(53, 83);

    private double gateIntakeStartHeading = Math.toRadians(211);
    private Pose gateOpenPose = new Pose(17, 70, Math.toRadians(180));

    private Pose gateOpenControl = new Pose(48, 69);

    private Pose gateControl = new Pose(19, 60);

    private Pose gateControlFirst = new Pose(28, 50);

    private Pose gateIntakePose = new Pose(11, 57.8, Math.toRadians(152));

    private Pose closeSpikePose = new Pose(18, 84, Math.toRadians(180));
    private Pose endPose = new Pose(24, 80, Math.toRadians(180));

    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
            preloadHeading = MathUtils.mirrorHeading(preloadHeading);
            middleSpikePose = middleSpikePose.mirror();
            middleSpikeControl = middleSpikeControl.mirror();
            scoreMiddleSpikeControl = scoreMiddleSpikeControl.mirror();
            scorePose = scorePose.mirror();
            gateIntakeStartHeading = MathUtils.mirrorHeading(gateIntakeStartHeading);
            gateOpenPose = gateOpenPose.mirror();
            gateOpenControl = gateOpenControl.mirror();
            gateControl = gateControl.mirror();
            gateControlFirst = gateControlFirst.mirror();
            gateIntakePose = gateIntakePose.mirror();
            closeSpikePose = closeSpikePose.mirror();
            endPose = endPose.mirror();
        }
    }

    private PathChain scorePreload, intakeMiddleSpike, scoreMiddleSpike, openFirstGate, openGate, gateIntake, gateIntakeOld, scoreGate, intakeCloseSpike, scoreCloseSpike, parkPath;

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

        openFirstGate = follower().pathBuilder()
                .addPath(new BezierCurve(middleSpikePose, gateControlFirst, gateOpenPose))
                .setConstantHeadingInterpolation(gateOpenPose.getHeading())
                .build();

        scoreMiddleSpike = follower().pathBuilder()
                .addPath(new BezierCurve(gateOpenPose, scoreMiddleSpikeControl, scorePose))
                .setLinearHeadingInterpolation(middleSpikePose.getHeading(), gateIntakeStartHeading)
                .build();

        openGate = follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, gateOpenControl, gateOpenPose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.5,
                                        HeadingInterpolator.tangent
                                ),
                            new HeadingInterpolator.PiecewiseNode(
                                    0.5,
                                    1,
                                    HeadingInterpolator.constant(gateOpenPose.getHeading())
                            )
                        )
                )
                .build();

        gateIntake = follower().pathBuilder()
                .addPath(new BezierCurve(gateOpenPose, gateControl, gateIntakePose))
                .setConstantHeadingInterpolation(gateIntakePose.getHeading())
                .build();

        gateIntakeOld = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, gateIntakePose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.5,
                                        HeadingInterpolator.constant(gateIntakeStartHeading)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.5,
                                        1,
                                        HeadingInterpolator.linear(gateIntakeStartHeading, gateIntakePose.getHeading())
                                )
                        )
                )
                .addParametricCallback(0.5, () -> follower().setMaxPower(0.4))
                .addParametricCallback(0.8, () -> follower().setMaxPower(1))
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
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.3,
                                        HeadingInterpolator.constant(closeSpikePose.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.3,
                                        1,
                                        HeadingInterpolator.linear(closeSpikePose.getHeading(), gateIntakeStartHeading)
                                )
                        )
                )
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
                new ParallelRaceGroup(
                        new SequentialGroup(
                                new FollowPath(intakeMiddleSpike),
                                Intake.INSTANCE.stopIntake,
                                new FollowPath(openFirstGate)
                        ),
                        new Delay(7)
                ),


                // Score Middle Spike
                new ParallelRaceGroup(
                        new SequentialGroup(
                                new ParallelGroup(
                                        new FollowPath(scoreMiddleSpike),
                                        Intake.INSTANCE.openGate
                                ),
                                shootArtifacts()
                                ),
                        new Delay(4.5)
                ),


                // Gate Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(openGate),
                new ParallelRaceGroup(
                        new SequentialGroup(
                                new FollowPath(gateIntake),
                                new ParallelRaceGroup(
                                        new Delay(GATE_DELAY),
                                        new WaitUntil(() -> Intake.INSTANCE.hasThreeBalls)
                                )
                        ),
                        new Delay(2)
                ),
                Intake.INSTANCE.stopIntake,

                // Score Gate
                new InstantCommand(() -> follower().setMaxPower(1)),
                new ParallelRaceGroup(
                        new SequentialGroup(
                                new ParallelGroup(
                                        new FollowPath(scoreGate),
                                        Intake.INSTANCE.openGate
                                ),
                                shootArtifacts()
                        ),
                        new Delay(4.5)
                ),


                // Intake Close Spike
                new ParallelRaceGroup(
                        new SequentialGroup(
                            Intake.INSTANCE.intakeArtifacts,
                            new FollowPath(intakeCloseSpike),
                            Intake.INSTANCE.stopIntake
                        ),
                        new Delay(3)
                ),

                // Score Close Spike
                new ParallelRaceGroup(
                        new SequentialGroup(
                            new ParallelGroup(
                                    new FollowPath(scoreCloseSpike),
                                    Intake.INSTANCE.openGate
                            ),
                            shootArtifacts()
                        ),
                        new Delay(4.5)
                ),

                // Gate Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(openGate),
                new ParallelRaceGroup(
                        new SequentialGroup(
                                new FollowPath(gateIntake),
                                new ParallelRaceGroup(
                                        new Delay(GATE_DELAY),
                                        new WaitUntil(() -> Intake.INSTANCE.hasThreeBalls)
                                )
                        ),
                        new Delay(2)
                ),
                Intake.INSTANCE.stopIntake,

                // Score Gate
                new InstantCommand(() -> follower().setMaxPower(1)),
                new ParallelRaceGroup(
                        new SequentialGroup(
                            new ParallelGroup(
                                    new FollowPath(scoreGate),
                                    Intake.INSTANCE.openGate
                            ),
                            shootArtifacts()
                        ),
                        new Delay(4.5)
                ),

                // Gate Intake
                Intake.INSTANCE.intakeArtifacts,
                new FollowPath(openGate),
                new ParallelRaceGroup(
                        new SequentialGroup(
                                new FollowPath(gateIntake),
                                new ParallelRaceGroup(
                                        new Delay(GATE_DELAY),
                                        new WaitUntil(() -> Intake.INSTANCE.hasThreeBalls)
                                )
                        ),
                        new Delay(2)
                ),
                Intake.INSTANCE.stopIntake,

                // Score Gate
                new InstantCommand(() -> follower().setMaxPower(1)),
                new ParallelRaceGroup(
                        new SequentialGroup(
                                new ParallelGroup(
                                        new FollowPath(scoreGate),
                                        Intake.INSTANCE.openGate
                                ),
                                shootArtifacts()
                        ),
                        new Delay(4.5)
                ),

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
        telemetry.addData("MaxPower", follower().getMaxPowerScaling());
        telemetry.addData("Param Path Pos", follower().getPathCompletion());
        telemetry.update();
        LightingController.get().update();
    }

    @Override
    public void onStop() {
        Flywheel.INSTANCE.turnFlywheelOff.schedule();
        Turret.INSTANCE.disableTracking.schedule();
    }
}