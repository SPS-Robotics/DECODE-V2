package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.RobotState;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.CloseGate15;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.ThreeSpike12;

@Autonomous(name = "12 Artifact - All 3 Spikes - RED", group = "12 Artifact - All 3 Spikes")
public class Red12 extends ThreeSpike12 {
    public Red12() {
        super(RobotState.AllianceColor.RED);
    }
}
