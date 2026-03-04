package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class MathUtils {
    public static double calculateAngleToPose(Pose robot, Pose target) {
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();

        double rawDelta = Math.atan2(dy, dx) - robot.getHeading();

        return Math.atan2(Math.sin(rawDelta), Math.cos(rawDelta));
    }

    public static double clampValue(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}
