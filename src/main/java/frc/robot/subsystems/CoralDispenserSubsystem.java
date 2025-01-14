package frc.robot.subsystems;

import org.apache.logging.log4j.CloseableThreadContext.Instance;

import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class CoralDispenserSubsystem extends SubsystemBase {
    private FridoSparkMax coralMotorTop;
    private FridoSparkMax coralMotorChangePitch;

    private PidValues pidValues = new PidValues(0, 0, 0, 0); // p, i, d, f
    
    public CoralDispenserSubsystem() {
        coralMotorTop = new FridoSparkMax(Constants.CoralDispenser.coralMotorTopID);
        
        coralMotorChangePitch = new FridoSparkMax(Constants.ClimberSubsytem.coralMotorChangePitchID);
        
        coralMotorChangePitch.setPID(pidValues);
        setPitch(0);
    }

    public void setMotorTopSpeed(double speed) {
        if (speed > 1 || speed < -1) {
            throw new IllegalArgumentException("Speed must be between -1 and 1");
        }
        coralMotorTop.set(speed);
    }

    public void stopMotorTop() {
        coralMotorTop.set(0);
    }

    /**
     * 
     * @param pitch in degrees
     */
    public void setPitch(double pitch) {
        Rotation2d rotation = Rotation2d.fromDegrees(pitch);
        coralMotorChangePitch.setPosition(rotation.getRotations());
    }
}
