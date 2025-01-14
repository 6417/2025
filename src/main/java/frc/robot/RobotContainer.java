package frc.robot;

import java.util.List;

import frc.robot.Constants.CoralDispenser;
import frc.robot.Constants.LiftingTower;
import frc.robot.subsystems.ClimberSubsytem;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.LiftingTowerSubsystem;
import frc.robot.swerve.SwerveDrive2024;

public class RobotContainer {
    public static final SwerveDrive2024 drive = new SwerveDrive2024();
    public static final Controls controls = new Controls();
    
    public static final ClimberSubsytem climber = new ClimberSubsytem();
    public static final CoralDispenserSubsystem coralDispenser = new CoralDispenserSubsystem();
    public static final LiftingTowerSubsystem liftingTower = new LiftingTowerSubsystem();
}
