package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class SoftwareConstants {
    public static class Poses {
        public static Pose blueGoalPose = new Pose(0, 144);
        public static Pose redGoalPose = new Pose(144, 144);

        public static Pose blueLoadingZoneCorner = new Pose();
        public static Pose redLoadingZoneCorner = new Pose();
    }

    public static class Turret {
        public static double ticksPerRevolution = 8192; // REV Through Bore Encoder
        public static double pulleyRatio = 1; // Large Pulley Teeth / Small Pulley Teeth

        public static double minTicks = -2000;
        public static double maxTicks = 2000;

        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
    }

    public static class Intake {
        public static double intakePower = 1;

        public static double gateOpenPosition = 0;
        public static double gateClosePosition = 1;
    }

    public static class Flywheel {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;

        public static double kV = 0;
        public static double kA = 0;
        public static double kS = 0;

        public static double shootingVelocity = 800;
    }
}
