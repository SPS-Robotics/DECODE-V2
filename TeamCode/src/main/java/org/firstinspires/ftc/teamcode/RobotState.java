package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class RobotState {
    public enum Alliance {
        BLUE, RED
    }

    public static Alliance alliance = Alliance.BLUE;
    public static Pose autoEndPose = new Pose();
}
