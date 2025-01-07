package frc.robot.abstraction;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import edu.wpi.first.units.Measure;
import frc.fridowpi.motors.utils.PidValues;

/**
 * RobotData: Holds important constants of the active robot.
 */
public record RobotData(
        HardwareData hardware,
        DriveData drive,
        AutoData auto,
        PidData pid) {

    public record HardwareData(
            double wheelCircumference,
            double trackWidth,
            double encoderToMeters) {
    }

    public enum MotorRole {
        LeftMaster, RightMaster,
        LeftFollower, RightFollower
    }

    public record DriveData(
            boolean enabled,
            List<Integer> motorIds,
            List<MotorRole> inverted) {
    }

    public record AutoData(
            double maxVelocity,
            double maxAcceleration,
            double maxTurnSpeed,
            double ksVolts,
            double kvVolts,
            double kaVolts,
            double kRamseteB,
            double kRamseteZeta) {
    }

    public record PidData(
            PidValues pathWeaver,
            PidValues driveLeft,
            PidValues driveRight) {
    }

    /* Constructors */
    public RobotData() {
        this(new HardwareData(
                0,
                0,
                0),
                new DriveData(
                        false,
                        List.of(),
                        List.of()),
                new AutoData(
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0),
                new PidData(
                        new PidValues(0, 0, 0),
                        new PidValues(0, 0, 0),
                        new PidValues(0, 0, 0)));
    }
}
