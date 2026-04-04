package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals.Constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight() { }

    private boolean tryRelocalise = false;

    private Limelight3A ll;
    private List<Pose> relocalisationPoses = new ArrayList<>();

    public Command relocaliseOdometry = new InstantCommand(() -> {
        relocalisationPoses.clear();
        tryRelocalise = true;
    });

    private Pose convertLLToPose(Pose3D limelightPose) {
        double xInches = limelightPose.getPosition().x * Constants.Limelight.METERS_TO_INCHES;
        double yInches = limelightPose.getPosition().y * Constants.Limelight.METERS_TO_INCHES;

        return new Pose(yInches + 72, -xInches + 72, PedroComponent.follower().getHeading());
    }

    private List<Pose> filterPoses(List<Pose> relocalisationPoses) {
        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();

        for (Pose p : relocalisationPoses) {
            xs.add(p.getX());
            ys.add(p.getY());
        }

        Collections.sort(xs);
        Collections.sort(ys);

        Pose medianPose = new Pose(xs.get(xs.size() / 2), ys.get(ys.size() / 2));

        List<Pose> filteredPoses = new ArrayList<>();

        for (Pose p : relocalisationPoses) {
            if (p.distanceFrom(medianPose) < Constants.Limelight.MAX_ERROR_INCH) filteredPoses.add(p);
        }

        return filteredPoses;
    }

    private Pose averagePoses(List<Pose> filteredPoses) {
        double sumX = 0;
        double sumY = 0;

        for (Pose p : filteredPoses) {
            sumX += p.getX();
            sumY += p.getY();
        }

        return new Pose(sumX / filteredPoses.size(), sumY / filteredPoses.size(), PedroComponent.follower().getHeading());
    }

    @Override
    public void initialize() {
        ll = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        ll.setPollRateHz(50);
        ll.pipelineSwitch(0); // Relocalisation Pipeline by default.

        ll.start();
    }

    public Pose latest = new Pose();

    @Override
    public void periodic() {
        if (tryRelocalise) {
            LLResult result = ll.getLatestResult();
            if (result == null || !result.isValid()) return;

            if (Math.abs(PedroComponent.follower().getAngularVelocity()) > Constants.Limelight.MAX_RELOC_VELOCITY) {
                tryRelocalise = false;
                relocalisationPoses.clear();
            }

            Pose3D limelightPose = result.getBotpose();

            relocalisationPoses.add(convertLLToPose(limelightPose));
        }

        if (relocalisationPoses.size() >= 50) {
            tryRelocalise = false;
            List<Pose> filteredPoses = filterPoses(relocalisationPoses);

            latest = averagePoses(filteredPoses);

            if (!filteredPoses.isEmpty()) PedroComponent.follower().setPose(latest);

            relocalisationPoses.clear();
        }

    }
}
