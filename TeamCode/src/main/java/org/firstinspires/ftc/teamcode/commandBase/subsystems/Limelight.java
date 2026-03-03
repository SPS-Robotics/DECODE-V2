package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

public class Limelight implements Subsystem {
    public static Limelight INSTANCE = new Limelight();

    private Limelight() { }

    private Limelight3A limelight3A;

    @Override
    public void initialize() {
        limelight3A = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(50);
        limelight3A.pipelineSwitch(0);

        limelight3A.start();
    }

    private boolean tryRelocalise = false;

    @Override
    public void periodic() {
        if (!tryRelocalise) return;

        LLResult result = limelight3A.getLatestResult();
        if (result == null || !result.isValid()) return;

        Pose3D limelightPose = result.getBotpose();

    }
}
