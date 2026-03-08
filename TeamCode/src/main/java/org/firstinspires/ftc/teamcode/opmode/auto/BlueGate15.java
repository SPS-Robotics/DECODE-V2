package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.CloseGate15;


@Autonomous(name = "15 Artifact - Close Gate Intake - BLUE")
public class BlueGate15 extends CloseGate15 {
    public BlueGate15() {
        super(RobotState.AllianceColor.BLUE);
    }
}
