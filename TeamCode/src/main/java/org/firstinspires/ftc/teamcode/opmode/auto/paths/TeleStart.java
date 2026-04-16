package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.teamcode.commandBase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

public abstract class TeleStart extends NextFTCOpMode {
    protected final RobotState.AllianceColor alliance;

    public TeleStart(RobotState.AllianceColor alliance) {
        addComponents(new PedroComponent(Constants::createFollower), new SubsystemComponent(Turret.INSTANCE));
        this.alliance = alliance;
    }

    private Pose startPose = new Pose(14.0, 112.093, Math.toRadians(270));

    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
        }
    }


    @Override
    public void onInit() {
        RobotState.setAlliance(alliance);
        initPoses();
        follower().setStartingPose(startPose);
        Turret.INSTANCE.setTurretPosition(0);
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Alliance Colour", RobotState.ALLIANCE_COLOR);
    }
}
