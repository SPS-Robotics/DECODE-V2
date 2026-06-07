package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.CloseGate18;

@Autonomous(name = "18 Artifacts Gate Intake - RED", group = "18 Artifact - Close Gate Intake")

public class RedCloseGate18 extends CloseGate18 {
    public RedCloseGate18() { super(RobotState.AllianceColor.RED); }
}
