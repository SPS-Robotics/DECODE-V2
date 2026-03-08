package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.TeleStart;

@Autonomous(name="Blue Teleop Start Position")
public class BlueTeleStart extends TeleStart {
    public BlueTeleStart() {
        super(RobotState.AllianceColor.BLUE);
    }
}
