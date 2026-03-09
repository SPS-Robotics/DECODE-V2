package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.CloseGate15;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.ThreeSpike12;

@Autonomous(name = "12 Artifact - All 3 Spikes - BLUE")
public class Blue12 extends ThreeSpike12 {
    public Blue12() {
        super(RobotState.AllianceColor.BLUE);
    }
}
