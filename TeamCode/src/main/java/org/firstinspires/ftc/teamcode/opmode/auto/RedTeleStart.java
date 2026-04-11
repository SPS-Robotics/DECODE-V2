package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.TeleStart;

@Autonomous(name = "Red Teleop Start Position", group = "Close No-Auto Start")
public class RedTeleStart extends TeleStart {
    public RedTeleStart() {
        super(RobotState.AllianceColor.RED);
    }
}
