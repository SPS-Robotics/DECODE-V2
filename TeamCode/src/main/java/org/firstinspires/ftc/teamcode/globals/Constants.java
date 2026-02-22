package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Constants {
    public static class Turret {
        public static double ticksPerRevolution = 28.0 * 18.9; // Motor Encoder Calculation Required
        public static double pulleyRatio = 107.0/24.0; // Large Pulley Teeth / Small Pulley Teeth

        public static double MIN_TICKS = -1000;
        public static double MAX_TICKS = 1000;

        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
    }

    public static class Intake {
        public static double INTAKE_POWER = 1;

        public static double GATE_OPEN = 0;
        public static double GATE_CLOSE = 1;
    }

    public static class Flywheel {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;

        public static double kV = 0;
        public static double kA = 0;
        public static double kS = 0;

        public static double IDLE_VELOCITY = 100;
    }
}
