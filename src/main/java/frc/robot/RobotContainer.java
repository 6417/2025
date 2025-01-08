package frc.robot;

import java.util.List;

import frc.robot.abstraction.RobotData;
import frc.robot.abstraction.RobotData.AutoData;
import frc.robot.abstraction.RobotData.DriveData;
import frc.robot.abstraction.RobotData.HardwareData;
import frc.robot.abstraction.RobotData.PidData;
import frc.robot.swerve.SwerveDrive2024;

public class RobotContainer {
    public static class Container2024 {
        public final RobotData data = new RobotData(
                new HardwareData(
                        Double.NaN,
                        0.12 * Math.PI,
                        1 / Constants.SwerveDrive.Swerve2024.metersToRelativeEncoder),
                new DriveData(
                        Constants.SwerveDrive.Swerve2024.enabled,
                        List.of(),
                        List.of()),
                new AutoData(
                        Constants.SwerveDrive.Swerve2024.maxVelocity,
                        Constants.SwerveDrive.Swerve2024.maxAcceleration,
                        Constants.SwerveDrive.Swerve2024.maxTurnSpeed,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN),
                new PidData(null, null, null));

        public final SwerveDrive2024 swerve = new SwerveDrive2024();
    }

    public static final Container2024 mechanisms = new Container2024();
    public static final Controls controls = new Controls();

    public static RobotData data() {
        return mechanisms.data;
    }
}
