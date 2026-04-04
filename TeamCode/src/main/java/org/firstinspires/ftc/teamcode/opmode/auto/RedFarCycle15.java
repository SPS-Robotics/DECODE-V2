package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.FarCycle15;

@Autonomous(name="15 Artifacts Far Cycling - RED")
public class RedFarCycle15 extends FarCycle15 {
    public RedFarCycle15() { super(RobotState.AllianceColor.RED); }
}
