package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.FarPush15;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.ThreeSpike12;

@Autonomous(name = "15 Artifact - ALLIANCE PUSHER - BLUE", group = "15 Artifact - ALLIANCE PUSHER")
public class BlueFarPush extends FarPush15 {
    public BlueFarPush() {
        super(RobotState.AllianceColor.BLUE);
    }
}
