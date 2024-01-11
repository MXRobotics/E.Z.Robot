package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.common.centerstage.Side;

public class Globals {
    public static Side SIDE = Side.LEFT;
    public static Side ALLIANCE = Side.BLUE;

    public static boolean IS_AUTO = false;
    public static boolean IS_TELEOP = true;
    public static boolean IS_TESTING = false;

    public static boolean USING_DASHBOARD = false;

    public static boolean IS_SCORING = false;
    public static boolean IS_INTAKING = false;

    public static void startScoring() {
        IS_SCORING = true;
        IS_INTAKING = false;
    }

    public static void startIntaking() {
        IS_SCORING = false;
        IS_INTAKING = true;
    }

    public static void stopIntaking() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }

    public static void stopScoring() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }


}
