package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class MathUtils {
    public static double calculateAngleToPose(Pose robot, Pose target) {
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();

        double rawDelta = Math.atan2(dy, dx) - (robot.getHeading() + Math.PI);

        return Math.atan2(Math.sin(rawDelta), Math.cos(rawDelta));
    }

    public static Pose translatePose(Pose original, double distance) {
        Vector displacement = original.getHeadingAsUnitVector().times(distance);
        return original.plus(new Pose(
                displacement.getXComponent(),
                displacement.getYComponent(),
                0
        ));
    }

    public static double clampValue(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    public static double mirrorHeading(double heading) {
        return Math.PI - heading;
    }
}
