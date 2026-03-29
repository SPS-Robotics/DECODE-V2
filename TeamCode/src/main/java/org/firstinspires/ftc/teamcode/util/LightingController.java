package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.Prism.GoBildaPrismDriver.Artboard;

import org.firstinspires.ftc.teamcode.util.Prism.GoBildaPrismDriver;

import dev.nextftc.ftc.ActiveOpMode;

public final class LightingController {
    private static LightingController instance;

    private final GoBildaPrismDriver prism;

    private Artboard last;
    private boolean hasThreeBalls;

    private LightingController() {
        prism = ActiveOpMode.hardwareMap()
                .get(GoBildaPrismDriver.class, "prism");
    }

    public static void init() {
        if (instance == null) {
            instance = new LightingController();
        }
    }


    public void update() {
        Artboard next = resolve();

        if (next != last) {
            LightingController.get().getPrism().loadAnimationsFromArtboard(next);
            last = next;
        }
    }

    public static LightingController get() {
        if (instance == null) {
            instance = new LightingController();
        }
        return instance;
    }

    private GoBildaPrismDriver.Artboard resolve() {
        if (hasThreeBalls) {
            return Artboard.ARTBOARD_1;
        }
        return Artboard.ARTBOARD_0;
    }

    public void setRobotFull(boolean input) {
        hasThreeBalls = input;
    }

    public GoBildaPrismDriver getPrism() {
        return prism;
    }
}
