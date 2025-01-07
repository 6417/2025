package frc.robot.abstraction.baseClasses;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import frc.fridowpi.joystick.Binding;
import frc.fridowpi.module.Module;
import frc.robot.RobotContainer;
import frc.robot.abstraction.interfaces.IDrive;

/**
 * BDrive: Abstract base class for all drive subsystems
 **/
public abstract class BDrive extends Module implements IDrive {

    public enum DriveOrientation {
        FieldOriented, Forwards, Backwards
    }

    public enum MountingLocations {
        FrontRight, FrontLeft, BackRight, BackLeft
    }

    public enum SpeedFactor {
        DEFAULT_SPEED, SLOW, FAST;
    }

    protected DriveOrientation driveOrientation = DriveOrientation.FieldOriented;

    @Override
    public final DriveOrientation getOrientation() {
        return driveOrientation;
    }

    @Override
    public void setOrientation(DriveOrientation driveMode) {
        this.driveOrientation = driveMode;
    }

    @Override
    public List<Binding> getMappings() {
        return List.of();
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public double percent2rotationVelocityDouble(double val) {
        // Attention: For swerves the turning speed of the single wheels?
        return RobotContainer.data().auto().maxTurnSpeed() * val;
    }

    @Override
    public double percent2rotationVelocity(double val) {
        // Attention: For swerves the turning speed of the single wheels?
        return RobotContainer.data().auto().maxTurnSpeed() * val;
    }

    @Override
    public double percent2driveVelocity(double x) {
        return RobotContainer.data().auto().maxVelocity() * x;
    }
}
