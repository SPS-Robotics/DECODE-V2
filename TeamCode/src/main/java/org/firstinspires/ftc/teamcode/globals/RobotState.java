package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.geometry.Pose;

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
    public static Pose AUTO_END_POSE = new Pose();

    public static Pose GOAL_POSE = new Pose((72 + (65*ALLIANCE_COLOR.getMultiplier())), 140); // tune and then do shooter and hood tuning.
    public static Pose LOADING_ZONE = new Pose(72 + (72 * ALLIANCE_COLOR.getMultiplier()), 0); // depends on robot centre - tune later.
}
