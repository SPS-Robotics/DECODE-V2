package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.FarCycle15;

@Autonomous(name="15 Artifacts Far Cycling - BLUE")
public class BlueFarCycle15 extends FarCycle15 {
    public BlueFarCycle15() { super(RobotState.AllianceColor.BLUE); }
}
