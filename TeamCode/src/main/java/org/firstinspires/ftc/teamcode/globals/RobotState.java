package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

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
    public static Pose AUTO_END_POSE = new Pose(17.5, 120, Math.toRadians(324));

    public static Pose GOAL_POSE = new Pose((72 + (70*ALLIANCE_COLOR.getMultiplier())), 133); // tune and then do shooter and hood tuning.
    public static Pose LOADING_ZONE = new Pose(72 + (72 * ALLIANCE_COLOR.getMultiplier()), 0); // depends on robot centre - tune later.

    public static double TURRET_END_POS = 0;

    public static void setAlliance(AllianceColor alliance) {
        ALLIANCE_COLOR = alliance;
        GOAL_POSE = new Pose((72 + (70*alliance.getMultiplier())), 133);
        LOADING_ZONE = new Pose(72 + (72 * alliance.getMultiplier()), 0);
        if (alliance == AllianceColor.BLUE) AUTO_END_POSE = new Pose(17.5, 120, Math.toRadians(324));
        else AUTO_END_POSE = new Pose(17.5, 120, Math.toRadians(324)).mirror();
        TURRET_END_POS = 0;
    }
}
