package frc.robot.subsystems;

import org.apache.logging.log4j.CloseableThreadContext.Instance;

import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.robot.Constants;

public class CoralDispenserSubsystem extends SubsystemBase {
    private FridoSparkMax coralMotorTop;
    private FridoSparkMax coralMotorBottomMaster;
    private static CoralDispenserSubsystem instance;

    public CoralDispenserSubsystem() {
        coralMotorTop = new FridoSparkMax(Constants.CoralDispenser.coralMotorTopID);
        coralMotorBottomMaster = new FridoSparkMax(Constants.CoralDispenser.coralMotorBottomID);

        coralMotorBottomMaster.follow(coralMotorTop, DirectionType.invertMaster);
    }

    private CoralDispenserSubsystem() {
    
    }

    public static CoralDispenserSubsystem getInstance() {
        if (instance == null) {
            instance = new CoralDispenserSubsystem();
        }
        return instance;
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
}
