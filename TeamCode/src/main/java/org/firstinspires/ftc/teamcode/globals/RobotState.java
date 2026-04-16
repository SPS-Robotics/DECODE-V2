package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.util.InterpLUT;
import org.firstinspires.ftc.teamcode.util.MathUtils;

import java.util.Arrays;

import dev.nextftc.extensions.pedro.PedroComponent;

@Configurable
public class RobotState {
    public enum AllianceColor {
        BLUE(-1), RED(1);

        private final int val;

        AllianceColor(int multiplier) {
            val = multiplier;
        }

        public int getMultiplier() {
            return val;
        }
    }

    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static Pose GOAL_POSE = new Pose(2, 133);
    public static Pose LOADING_ZONE = new Pose(141.5 - 10.343, 8, Math.toRadians(0));

    public static Pose AUTO_END_POSE = new Pose(14.0, 112.093, Math.toRadians(270));

    public static double AUTO_END_X = 14.0;
    public static double AUTO_END_Y = 112.093;
    public static double AUTO_END_HEADING = Math.toRadians(270);
    public static double GATE_HEADING = Math.toRadians(150);
    public static double PARK_HEADING = Math.toRadians(225);

    public static double TURRET_END_POS = 0;

    /*
    public static final InterpLUT velocityLUT = new InterpLUT(
            Arrays.asList(41.611, 55.9399, 64.3681, 67.3679, 69.9892, 81.2249, 83.0044, 92.3683, 93.3369, 113.2097, 132.8782, 135.4035, 144.2914), // Distance
            Arrays.asList(1340.0, 1520.0, 1480.0, 1540.0, 1560.0, 1620.0, 1620.0, 1640.0, 1700.0, 1840.0, 2060.0, 2040.0, 2160.0) // Flywheel RPM
    ).createLUT();

    public static final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(41.611, 55.9399, 64.3681, 67.3679, 69.9892, 81.2249, 83.0044, 92.3683, 93.3369, 113.2097, 132.8782, 135.4035, 144.2914),
            Arrays.asList(0.64, 0.58, 0.50, 0.48, 0.34, 0.36, 0.34, 0.32, 0.30, 0.24, 0.28, 0.20, 0.20)
    ).createLUT();
     */

    public static final InterpLUT velocityLUT = new InterpLUT(
            Arrays.asList(31.2829, 39.368, 48.8716, 53.1186, 57.5015, 66.2944, 70.8819, 78.1916, 85.3848, 89.8278, 91.6035, 131.4509, 138.8672),
            Arrays.asList(1140.0, 1140.0, 1180.0, 1200.0, 1240.0, 1280.0, 1340.0, 1380.0, 1400.0, 1520.0, 1460.0, 1800.0, 1840.0)
    ).createLUT();

    public static final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(31.2829, 39.368, 48.8716, 53.1186, 57.5015, 66.2944, 70.8819, 78.1916, 85.3848, 89.8278, 91.6035, 131.4509, 138.8672),
            Arrays.asList(0.98, 0.86, 0.74, 0.68, 0.62, 0.58, 0.54, 0.48, 0.42, 0.38, 0.38, 0.30, 0.28)
    ).createLUT();

    public static final InterpLUT tofLUT = new InterpLUT(
            Arrays.asList(42.0971, 49.6647, 56.9606, 66.5632, 67.5254, 72.2892, 80.6825, 87.8641),
            Arrays.asList(1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7)
    ).createLUT();

    public static Pose velocityCompensate(Pose goalPose) {
        Pose robotPose = PedroComponent.follower().getPose();
        Vector velocity = PedroComponent.follower().getVelocity();

        Pose compensated = goalPose;
        for (int i = 0; i < 2; i++) {
            double distance = robotPose.distanceFrom(compensated);
            compensated = MathUtils.velocityCompensatePose(compensated, velocity, tofLUT.get(distance));
        }

        return compensated;
    }

    public static void setAlliance(AllianceColor alliance) {
        ALLIANCE_COLOR = alliance;
        TURRET_END_POS = 0;

        if (alliance == AllianceColor.BLUE) {
            GOAL_POSE = new Pose(2, 133);
            LOADING_ZONE = new Pose(141.5 - 10.343, 8, Math.toRadians(0));
            AUTO_END_POSE = new Pose(14.0, 112.093, Math.toRadians(270));
            AUTO_END_X = 14.0;
            AUTO_END_Y = 112.093;
            AUTO_END_HEADING = Math.toRadians(270);
            GATE_HEADING = Math.toRadians(156);
            PARK_HEADING = Math.toRadians(225);
        }

        if (alliance == AllianceColor.RED) {
            GOAL_POSE = new Pose(141.5 - 2, 133);
            LOADING_ZONE = new Pose(141.5 - 10.343, 8, Math.toRadians(0)).mirror();
            AUTO_END_POSE = new Pose(14.0, 112.093, Math.toRadians(270)).mirror();
            AUTO_END_X = 141.5-14.0;
            AUTO_END_Y = 112.093;
            AUTO_END_HEADING = MathUtils.mirrorHeading(Math.toRadians(270));
            GATE_HEADING = MathUtils.mirrorHeading(Math.toRadians(156));
            PARK_HEADING = MathUtils.mirrorHeading(Math.toRadians(225));
        }
    }
}
