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
    //
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static Pose GOAL_POSE = new Pose(3, 133);
    public static Pose LOADING_ZONE = new Pose(141.5 - 10.343, 8, Math.toRadians(0));

    public static Pose GATE_RELOC_POSE = new Pose(15.7, 78.0, Math.toRadians(90));

    public static Pose AUTO_END_POSE = new Pose(13.6, 111.4, Math.toRadians(0));

    public static double AUTO_END_X = 13.6;
    public static double AUTO_END_Y = 111.4;
    public static double AUTO_END_HEADING = Math.toRadians(0);
    public static double GATE_HEADING = Math.toRadians(150);
    public static double PARK_HEADING = Math.toRadians(225);

    public static double TURRET_END_POS = 0;

    public static boolean SOTM = false;

    public static final InterpLUT velocityLUT = new InterpLUT(
            Arrays.asList(41.021, 48.0373, 51.1171, 62.0988, 68.3172, 73.3488, 86.1183, 97.4331, 127.7073, 129.9429, 137.5253, 142.0984),
            Arrays.asList(1180.0, 1180.0, 1220.0, 1280.0, 1320.0, 1340.0, 1460.0, 1540.0, 1820.0, 1860.0, 1880.0, 1900.0)
    ).createLUT();

    public static final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(41.021, 48.0373, 51.1171, 62.0988, 68.3172, 73.3488, 86.1183, 97.4331, 127.7073, 129.9429, 137.5372, 142.0984),
            Arrays.asList(0.60, 0.58, 0.56, 0.50, 0.44, 0.42, 0.30, 0.24, 0.16, 0.18, 0.16, 0.14)
    ).createLUT();

    public static final double tofConst = 0.3;

    public static Pose velocityCompensate(Pose goalPose) {
        Pose robotPose = PedroComponent.follower().getPose();
        Vector velocity = PedroComponent.follower().getVelocity();

        Pose compensated = goalPose;
        for (int i = 0; i < 2; i++) {
            compensated = MathUtils.velocityCompensatePose(compensated, velocity, tofConst);
        }

        return compensated;
    }

    public static void setAlliance(AllianceColor alliance) {
        ALLIANCE_COLOR = alliance;
        TURRET_END_POS = 0;
        SOTM = false;

        if (alliance == AllianceColor.BLUE) {
            GOAL_POSE = new Pose(3, 133);
            LOADING_ZONE = new Pose(141.5 - 10.343, 8, Math.toRadians(0));
            AUTO_END_POSE = new Pose(13.6, 111.4, Math.toRadians(0));
            AUTO_END_X = 13.6;
            AUTO_END_Y = 111.4;
            AUTO_END_HEADING = Math.toRadians(0);
            GATE_HEADING = Math.toRadians(156);
            PARK_HEADING = Math.toRadians(225);
            GATE_RELOC_POSE = new Pose(15.7, 78.0, Math.toRadians(90));

        }

        if (alliance == AllianceColor.RED) {
            GOAL_POSE = new Pose(3, 133).mirror();
            LOADING_ZONE = new Pose(141.5 - 10.343, 8, Math.toRadians(0)).mirror();
            AUTO_END_POSE = new Pose(13.6, 111.4, Math.toRadians(0)).mirror();
            AUTO_END_X = 141.5-13.6;
            AUTO_END_Y = 111.4;
            AUTO_END_HEADING = MathUtils.mirrorHeading(Math.toRadians(0));
            GATE_HEADING = MathUtils.mirrorHeading(Math.toRadians(156));
            PARK_HEADING = MathUtils.mirrorHeading(Math.toRadians(225));
            GATE_RELOC_POSE = new Pose(15.7, 78.0, Math.toRadians(90)).mirror();
        }
    }
}
