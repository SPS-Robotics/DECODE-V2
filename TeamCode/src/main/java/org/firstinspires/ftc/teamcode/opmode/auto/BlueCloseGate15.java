package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.CloseGate15;

@Autonomous(name="15 Artifacts Gate Intake - BLUE")
public class BlueCloseGate15 extends CloseGate15 {
    public BlueCloseGate15() { super(RobotState.AllianceColor.BLUE); }
}
