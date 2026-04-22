package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.feedback.PIDCoefficients;

@Configurable
public class Constants {
    public static class Turret {
        public static double ticksPerRevolution = 28.0 * (76.0 / 21.0) * (84.0 / 29.0); // Motor Encoder Calculation Required
        public static double pulleyRatio = 109.0/24.0; // Large Pulley Teeth / Small Pulley Teeth

        public static double MIN_TICKS = -400;
        public static double MAX_TICKS = 600;

        public static double kP = 0.015;
        public static double kI = 0;
        public static double kD = 0.0003;

        public static double RELOC_POS = 55;

        public static double CENTRE_OFFSET = 110.0 / 127.0; // 22mm in inches

        public static double angularVelocitykV = 0.2;
    }

    public static class Intake {
        public static double INTAKE_POWER = 1;
        public static double OUTTAKE_POWER = -0.6;

        public static double GATE_OPEN = 0.95;
        public static double GATE_CLOSE = 0.26;

        public static double GATE_OPEN_TIME = 0.8;
    }

    public static class Flywheel {
        public static double kP = 0.007;
        public static double kI = 0;
        public static double kD = 0;

        public static double kV = 0.000379;
        public static double kA = 0;
        public static double kS = 0.05;

        // hood servo top = 0.08, hood servo bottom = 0.6
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
        public static double SHOOT_TIME = 0.75;
        public static double GATE_DELAY = 1.8;
    }

    public static class Limelight {
        public static double MAX_RELOC_VELOCITY = 1;
        public static double METERS_TO_INCHES = 39.3701;
        public static double MAX_ERROR_INCH = 1;
    }

    public static class Drive {
        public static double slowModeScalar = 0.2;
        public static PIDCoefficients mainCoefficients = new PIDCoefficients(0, 0, 0);
    }


}
