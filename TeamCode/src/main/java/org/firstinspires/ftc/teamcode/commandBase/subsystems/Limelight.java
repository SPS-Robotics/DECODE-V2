package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight() { }

    private Limelight3A limelight;

    private double[] getLLPython() {
        double[] pythonOutputs = {};

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) { pythonOutputs = result.getPythonOutput(); }

        return pythonOutputs;
    }

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double[] pythonOutputs = result.getPythonOutput();

        }
    }


}
