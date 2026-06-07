package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.CloseGate18;

@Autonomous(name = "18 Artifacts Gate Intake - BLUE", group = "18 Artifact - Close Gate Intake")
public class BlueCloseGate18 extends CloseGate18 {
    public BlueCloseGate18() { super(RobotState.AllianceColor.BLUE); }
}
