package org.firstinspires.ftc.teamcode.opmode.tuning;

import static org.firstinspires.ftc.teamcode.globals.RobotState.ALLIANCE_COLOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.globals.RobotState;


@TeleOp(name="Alliance Selector")
public class AllianceSelector extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Cross / Triangle", "Blue");
        telemetry.addData("Circle / Square", "Red");
        telemetry.addData("Alliance Color", ALLIANCE_COLOR);

        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.cross || gamepad2.cross || gamepad1.triangle || gamepad2.triangle) {
                ALLIANCE_COLOR = RobotState.AllianceColor.BLUE;
            }
            else if (gamepad1.circle || gamepad2.circle || gamepad1.square || gamepad2.square) {
                ALLIANCE_COLOR = RobotState.AllianceColor.RED;
            }

            telemetry.addData("Cross / Triangle", "Blue");
            telemetry.addData("Circle / Square", "Red");
            telemetry.addData("Alliance Color", ALLIANCE_COLOR);

            telemetry.update();
        }
    }
}
