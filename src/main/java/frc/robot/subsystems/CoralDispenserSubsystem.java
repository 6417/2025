package frc.robot.subsystems;

import org.apache.logging.log4j.CloseableThreadContext.Instance;

import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class CoralDispenserSubsystem extends SubsystemBase {
    private FridoSparkMax coralMotorTop;
    private FridoSparkMax coralMotorBottomMaster;
    private FridoSparkMax coralMotorChangePitch;

    private PidValues pidValues = new PidValues(0, 0, 0, 0); // p, i, d, f
    
    public CoralDispenserSubsystem() {
        coralMotorTop = new FridoSparkMax(Constants.CoralDispenser.coralMotorTopID);
        coralMotorBottomMaster = new FridoSparkMax(Constants.CoralDispenser.coralMotorBottomID);
        
        coralMotorChangePitch = new FridoSparkMax(Constants.ClimberSubsytem.coralMotorChangePitchID);
        
        coralMotorBottomMaster.follow(coralMotorTop, DirectionType.invertMaster);

        coralMotorChangePitch.setPID(pidValues);
    }

    public void setMotorSpeed(double speed) {
        if (speed > 1 || speed < -1) {
            throw new IllegalArgumentException("Speed must be between -1 and 1");
        }
        coralMotorBottomMaster.set(speed);
    }

    public void stopMotors() {
        coralMotorBottomMaster.set(0);
    }

    public void setPitch(double pitch) {
        coralMotorChangePitch.setPosition(pitch);
    }
}
