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
    public static Pose LOADING_ZONE = new Pose(141.5 - 7.95276, 6.90691968504, Math.toRadians(180));
    public static Pose AUTO_END_POSE = new Pose(17.5, 120, Math.toRadians(324));

    public static double AUTO_END_X = 17.5;
    public static double AUTO_END_Y = 120;
    public static double AUTO_END_HEADING = Math.toRadians(324);
    public static double GATE_HEADING = Math.toRadians(152);
    public static double PARK_HEADING = Math.toRadians(225);

    public static double TURRET_END_POS = 0;

    
    public static final InterpLUT velocityLUT = new InterpLUT(
            Arrays.asList(42.0971, 49.6647, 56.9606, 66.5632, 67.5254, 72.2892, 80.6825, 87.8641), // Distance
            Arrays.asList(1250.0, 1250.0, 1350.0, 1400.0, 1450.0, 1450.0, 1500.0, 1600.0) // Flywheel RPM
    ).createLUT();

    public static final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(42.0971, 49.6647, 56.9606, 66.5632, 67.5254, 72.2892, 80.6825, 87.8641),
            Arrays.asList(0.32, 0.30, 0.24, 0.20, 0.34, 0.20, 0.26, 0.18)
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
            LOADING_ZONE = new Pose(141.5 - 7.95276, 6.90691968504, Math.toRadians(180));
            AUTO_END_POSE = new Pose(17.5, 120, Math.toRadians(324));
            AUTO_END_X = 17.5;
            AUTO_END_Y = 120;
            AUTO_END_HEADING = Math.toRadians(324);
            GATE_HEADING = Math.toRadians(152);
            PARK_HEADING = Math.toRadians(225);
        }

        if (alliance == AllianceColor.RED) {
            GOAL_POSE = new Pose(141.5 - 2, 133);
            LOADING_ZONE = new Pose(141.5 - 7.95276, 6.90691968504, Math.toRadians(180)).mirror();
            AUTO_END_POSE = new Pose(17.5, 120, Math.toRadians(324)).mirror();
            AUTO_END_X = 141.5-17.5;
            AUTO_END_Y = 120;
            AUTO_END_HEADING = MathUtils.mirrorHeading(Math.toRadians(324));
            GATE_HEADING = MathUtils.mirrorHeading(Math.toRadians(-62));
            PARK_HEADING = Math.toRadians(315);
        }
    }
}
