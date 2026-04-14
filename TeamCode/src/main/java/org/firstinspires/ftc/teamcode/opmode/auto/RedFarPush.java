package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.FarPush15;

@Autonomous(name = "15 Artifact - ALLIANCE PUSHER - RED", group = "15 Artifact - ALLIANCE PUSHER")
public class RedFarPush extends FarPush15 {
    public RedFarPush() {
        super(RobotState.AllianceColor.RED);
    }
}
