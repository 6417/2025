package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.sensors.AnalogEncoder;
import frc.robot.Constants;
import frc.robot.swerve.SwerveUtils.CTREModuleState;
import frc.robot.swerve.SwerveUtils.SwerveModuleConstants;

public class SwerveModuleBase {
    private String mId;
    private int mModuleNumber;
    private  FridolinsMotor mDriveMotor;
    public FridolinsMotor mAngleMotor;
    public AnalogEncoder mRotEncoder;

    private SimpleMotorFeedforward mDriveFeedforward;

    private boolean isDriveMotorInverted = false;
    private boolean isAngleMotorInverted = true;
    private boolean isRotEncoderInverted = false;

    private double driveGearboxRatio = Constants.SwerveDrive.Swerve2024.driveGearRatio;
    private double angleGearboxRatio = Constants.SwerveDrive.Swerve2024.angleGearRatio;

    private double mWheelCircumference = Units.inchesToMeters(4) * Math.PI;

    private double maxSpeed = 5;

    private final double kAngleOffset;
    private double lastAngle;

    public SwerveModuleBase(int moduleNum, String name, SwerveModuleConstants constants) {

        mId = name;

        mModuleNumber = moduleNum;

        kAngleOffset = 0;

        mDriveFeedforward = new SimpleMotorFeedforward(constants.moduleTuningkS, constants.moduleTuningkV,
        0);

        mRotEncoder = new AnalogEncoder(constants.encoderID);
        mDriveMotor = new FridoFalcon500v6(constants.driveMotorID);
        mAngleMotor = new FridoSparkMax(constants.angleMotorID);


    }

    public void stop(){
        mDriveMotor.setVoltage(.0);
        mAngleMotor.setVoltage(.0);
    }

    public void setVoltage(double voltage) {
        mDriveMotor.setVoltage(voltage);
    }

    public void setNeutralMode2Brake(boolean brake) {
        mDriveMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void resetToAbsolute() {
        double position = mRotEncoder.getAbsolutePosition();
        mAngleMotor.setEncoderPosition(position);
    }

      public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // minimize the change in
                                                                                 // heading/easiest way

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
            mDriveMotor.set(percentOutput);
        } else {
            double desiredVelocity = (desiredState.speedMetersPerSecond / mWheelCircumference) * driveGearboxRatio;

            mDriveMotor.setVoltage(mDriveFeedforward.calculate(desiredVelocity)); // to be fixed
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // to be fixed

        //mAngleMotor.set(TalonFXControlMode.Position, Conversions.degreesToFalcon(lastAngle, angleGearbox.getRatio()));

        lastAngle = angle;

    }

    public int getModuleNumber() {
        return mModuleNumber;
    }

    public FridolinsMotor getDriveMotor() {
        return mDriveMotor;
    }

    public double getVelocityMPS() {
        return getVelocityRPM() * mWheelCircumference;
    }

    public double getVelocityRPM() {
        return getDriveMotor().getEncoderVelocity() / driveGearboxRatio;
    }

    public FridolinsMotor getAngleMotor() {
        return mAngleMotor;
    }

    public double getAngleOffset() {
        return kAngleOffset;
    }

    public String getName() {
        return mId;
    }

    public Rotation2d getEncoderRotation() {
        return Rotation2d.fromRotations(mRotEncoder.getAbsolutePosition());
    }

        public SwerveModulePosition getPosition() {
        double position = (mDriveMotor.getEncoderTicks() / driveGearboxRatio) * mWheelCircumference;
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getEncoderTicks());
        return new SwerveModulePosition(position, angle);
    }

    public SwerveModuleState getState() {
        double velocity = mDriveMotor.getEncoderVelocity() / driveGearboxRatio;
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getEncoderTicks());
        return new SwerveModuleState(velocity, angle);
    }


}