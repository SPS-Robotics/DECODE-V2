package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Constants {
    public static class Turret {
        public static double ticksPerRevolution = 28.0 * 18.9; // Motor Encoder Calculation Required
        public static double pulleyRatio = 107.0/24.0; // Large Pulley Teeth / Small Pulley Teeth

        public static double MIN_TICKS = -672;
        public static double MAX_TICKS = 875;

        public static double kP = 0.008;
        public static double kI = 0;
        public static double kD = 0.0001;
    }

    public static class Intake {
        public static double INTAKE_POWER = 1;
        public static double OUTTAKE_POWER = -0.4;

        public static double GATE_OPEN = 1;
        public static double GATE_CLOSE = 0;

        public static double GATE_OPEN_TIME = 0.6;
    }

    public static class Flywheel {
        public static double kP = 0.001;
        public static double kI = 0;
        public static double kD = 0;

        public static double kV = 0.00051;
        public static double kA = 0;
        public static double kS = 0.06;

        // hood servo top = 0.16, hood servo bottom = 0.68
    }

    public static class Lift {
        public static double LEFT_SERVO_ENGAGED = 0;
        public static double RIGHT_SERVO_ENGAGED = 1;

        public static double LEFT_SERVO_DISENGAGED = 1;
        public static double RIGHT_SERVO_DISENGAGED = 0;

        public static double ENGAGE_TIME = 1;
        public static double ENGAGING_POWER = 0.3;

        public static double LIFT_POWER = 1;
    }

    public static class Auto {
        public static double SHOOT_TIME = 1.5;
        public static double GATE_DELAY = 3;
    }
}
