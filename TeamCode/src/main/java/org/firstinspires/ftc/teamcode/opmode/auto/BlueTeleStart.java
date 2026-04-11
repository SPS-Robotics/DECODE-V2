package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.TeleStart;

@Autonomous(name = "Blue TeleOp Start Position", group = "Close No-Auto Start")
public class BlueTeleStart extends TeleStart {
    public BlueTeleStart() {
        super(RobotState.AllianceColor.BLUE);
    }
}
