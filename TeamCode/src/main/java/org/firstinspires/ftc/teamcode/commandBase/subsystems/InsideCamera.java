package org.firstinspires.ftc.teamcode.commandBase.subsystems;

import android.graphics.Canvas;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.InteriorArtifactProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class InsideCamera implements Subsystem {
    public static final InsideCamera INSTANCE = new InsideCamera();
    private InsideCamera() { }

    VisionProcessor interiorArtifactProcessor = new InteriorArtifactProcessor();

    VisionPortal visionPortal = new VisionPortal.Builder()
            .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "insideCamera"))
            .addProcessor(interiorArtifactProcessor)
            .setCameraResolution(new Size(480, 640))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build();
}
