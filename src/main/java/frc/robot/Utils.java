package frc.robot;

public class Utils {
    public static double normalizeAngleRad(double angle) {
        return Math.asin(Math.sin(angle));
    }
}
