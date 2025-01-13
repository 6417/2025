package frc.robot.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import frc.fridowpi.sensors.AnalogEncoder;

class ModuleConfig implements Cloneable {
    public String name;
    public double maxSpeed = 6.5;
    public double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
    public Translation2d moduleOffset;
    public double absEncoderOffset;

    public double driveGearboxRatio = 5.192;
    public int driveMotorID = 0;
    public double driveMotorStallCurrentLimit = 55;
    public double driveMotorFreeCurrentLimit = 30;
    public boolean isDriveMotorInverted = false;
    public PidValues drivePidValues = new PidValues(0.03, 0.00, 0);
    public FeedForwardValues driveFFValues = new FeedForwardValues(0.18, 0.27, 0);

    public int angleMotorID = 0;
    public double angleGearboxRatio = 47.62;
    public int angleMotorStallCurrentLimit = 35;
    public int angleMotorFreeCurrentLimit = 20;
    public double angleMotorIzone = 1.5;
    public boolean isAngleMotorInverted = false;
    public PidValues anglePidValues = new PidValues(1.05, 0.01, 1);

    public int encoderChannel = 0;
    public double encoderPositionOffset = 0;

    public double encoderThicksToRotationFalcon = 1;
    public double encoderVelocityToRPSFalcon = 1;
    public double encoderThicksToRotationNEO = 1;
    public double encoderVelocityToRPSNEO = 1;

    public FridolinsMotor makeDriveMotor() {
        FridoFalcon500v6 driveMotor = new FridoFalcon500v6(driveMotorID);
        driveMotor.factoryDefault();
        // driveMotor.asTalonFX().getConfigurator().apply(new
        // Slot0Configs().withKP(0.03).withKS(0.18).withKV(0.27));
        driveMotor.asTalonFX().getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(driveMotorFreeCurrentLimit)
                        .withSupplyCurrentLimit(driveMotorStallCurrentLimit));
        driveMotor.configEncoder(FridoFeedBackDevice.kBuildin, (int) encoderThicksToRotationFalcon);
        driveMotor.setInverted(isDriveMotorInverted);
        driveMotor.setPID(drivePidValues, driveFFValues);
        return driveMotor;
    }

    public FridolinsMotor makeAngleMotor() {
        FridoSparkMax angleMotor = new FridoSparkMax(angleMotorID);
        angleMotor.factoryDefault();
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(angleMotorStallCurrentLimit, angleMotorFreeCurrentLimit);
        config.closedLoop.iZone(angleMotorIzone);
        angleMotor.asSparkMax().configure(config, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        angleMotor.configEncoder(FridoFeedBackDevice.kBuildin, (int) encoderThicksToRotationNEO);
        angleMotor.setInverted(isAngleMotorInverted);
        angleMotor.setPID(anglePidValues);
        return angleMotor;
    }

    public AnalogEncoder makeAbsoluteEncoder() {
        AnalogEncoder encoder = new AnalogEncoder(encoderChannel);
        encoder.setPositionOffset(encoderPositionOffset);
        return encoder;
    }
}
