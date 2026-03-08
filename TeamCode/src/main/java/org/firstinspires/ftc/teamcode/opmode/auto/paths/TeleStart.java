package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

public abstract class TeleStart extends NextFTCOpMode {
    protected final RobotState.AllianceColor alliance;

    public TeleStart(RobotState.AllianceColor alliance) {
        addComponents(new PedroComponent(Constants::createFollower));
        this.alliance = alliance;
    }

    private Pose startPose = new Pose(17.5, 120, Math.toRadians(324));

    private void initPoses() {
        if (alliance == RobotState.AllianceColor.RED) {
            startPose = startPose.mirror();
        }
    }


    @Override
    public void onInit() {
        RobotState.setAlliance(alliance);
        initPoses();
        follower().setPose(startPose);
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Alliance Colour", RobotState.ALLIANCE_COLOR);
    }
}
