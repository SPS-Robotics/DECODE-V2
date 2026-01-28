package org.firstinspires.ftc.teamcode.util;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public abstract class InteriorArtifactProcessor implements VisionProcessor {

    private final int[][] xs = {
            {1, 2, 3},
            {2, 3, 4},
            {3, 4, 5}
    };

    private final int[][] ys = {
            {1, 2, 3},
            {2, 3, 4},
            {3, 4, 5}
    };

    private final int[][] background = new int[3][3];
    private boolean backgroundCaptured = false;

    boolean[] slotOccupied = new boolean[3];


    public enum StoredArtifacts {
        ONE,
        TWO,
        THREE
    }

    public void clearIntake() {
        slotOccupied = new boolean[] {false, false, false};
    }

    /*

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

     */


}
