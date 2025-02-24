package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class LiftingTowerSubsystem extends SubsystemBase {
    private final PidValues pidValues = Constants.LiftingTower.pidValues; // p, i, d, f
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.12, 0.25, 0.128, 0.05);
    private final TrapezoidProfile.Constraints constraints =  new TrapezoidProfile.Constraints(90, 200);

    private Debouncer debouncer;
    private SparkMaxConfig motorConfig;

    private FridoSparkMax motorSlave;
    private FridoSparkMax motorMaster;

    private double demandedHeight;
    private TrapezoidProfile.State desiredState;

    private TrapezoidProfile motionProfile = new TrapezoidProfile(constraints);;
    private Timer timer;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;


    public LiftingTowerSubsystem() {
        motorSlave = new FridoSparkMax(Constants.LiftingTower.liftingTowerRightId);
        motorMaster = new FridoSparkMax(Constants.LiftingTower.liftingTowerLeftId);

        motorConfig = new SparkMaxConfig();

        debouncer = new Debouncer(0.1, DebounceType.kRising);

        motorMaster.enableReverseLimitSwitch(Constants.LiftingTower.towerBottomSwitchPolarity, true);

        motorConfig.closedLoop.p(pidValues.kP)
                .i(pidValues.kI)
                .d(pidValues.kD)
                .outputRange(pidValues.peakOutputReverse, pidValues.peakOutputForward)
                .velocityFF(pidValues.kF.orElse(0.0));

        pidValues.iZone.ifPresent(iZone -> motorConfig.closedLoop.iZone(iZone));

        motorMaster.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        motorMaster.setIdleMode(IdleMode.kBrake);
        motorSlave.setIdleMode(IdleMode.kBrake);
        motorMaster.setInverted(true);

        SparkMaxConfig limitConfig = new SparkMaxConfig();
        limitConfig.softLimit
        .forwardSoftLimit(Constants.LiftingTower.softLimitTopPos).forwardSoftLimitEnabled(true);
        limitConfig.softLimit
        .reverseSoftLimit(Constants.LiftingTower.softLimitBottomPos).reverseSoftLimitEnabled(true);
    
        motorSlave.follow(motorMaster, DirectionType.invertMaster);

        motorMaster.asSparkMax().configure(limitConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        motorSlave.asSparkMax().configure(limitConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        demandedHeight = Constants.LiftingTower.resetEncoderPosition;
        desiredState = new TrapezoidProfile.State(0,0);

        timer = new Timer();

        timer.start();

        updateMotionProfile();
    }

    public void resetEncoder() {
        motorMaster.setEncoderPosition(Constants.LiftingTower.resetEncoderPosition);
    }

    public boolean isBottomSwitchPressed() {
        return motorMaster.isReverseLimitSwitchActive();
    }

    public void setPercent(double percent) {
        motorMaster.set(Math.max(Constants.LiftingTower.zeroingSpeed, percent));
    }

    @Override
    public void periodic() {
        if (debouncer.calculate(isBottomSwitchPressed())) {
            resetEncoder();
        }

        runAutomatic(); //ONLY FOR TESTING LATER MUST BE ADDED
    }

    public void setMotorSpeed(double speed) {
        motorMaster.set(speed);
    }

    public void setHeight(double desiredPosition){
        if(demandedHeight != desiredPosition){
            demandedHeight = desiredPosition;
            updateMotionProfile();
        }
    }

    private void updateMotionProfile(){
        startState = new TrapezoidProfile.State(motorMaster.getEncoderTicks(), motorMaster.getEncoderVelocity());
        endState = new TrapezoidProfile.State(demandedHeight, 0.0);
        timer.reset();
        desiredState = motionProfile.calculate(timer.get(), startState, endState);
    }

    static double clamp(double val, double min, double max)
    {
        if (val < min)
            val = min;
        if (val > max)
            val = max;
        return val;
    }


    public void runAutomatic(){ 
        double elapsedTime = timer.get();
        if (isAtDesiredHeight()){
            desiredState = new TrapezoidProfile.State(demandedHeight, 0.0);
        } else if (motionProfile.isFinished(elapsedTime)) {
            double vel = (demandedHeight - getHeight()) * 2;
            desiredState = new TrapezoidProfile.State(demandedHeight, vel);
        } else {
            desiredState = motionProfile.calculate(elapsedTime, startState, endState);
        }
        desiredState.position = clamp(desiredState.position, Constants.LiftingTower.softLimitBottomPos, Constants.LiftingTower.softLimitTopPos);
        desiredState.velocity = clamp(desiredState.velocity, -constraints.maxVelocity, constraints.maxVelocity);

        double ff = feedforward.calculate(desiredState.velocity);
        motorMaster.setPositionWithFeedforward(desiredState.position, ff);
    }

    public void stopMotors() {
        motorMaster.stopMotor();
    }

    public double getHeight(){
        return motorMaster.getEncoderTicks();
    }

    public boolean isAtDesiredHeight(){
        return Math.abs(demandedHeight - motorMaster.getEncoderTicks()) <= 1.5;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("targetState Velocity", () -> (desiredState.velocity * 60), null);
        builder.addDoubleProperty("currentState Velocity", motorMaster::getEncoderVelocity, null);
        builder.addDoubleProperty("targetState Position", () -> desiredState.position, null);
        builder.addDoubleProperty("currentState Positon", motorMaster::getEncoderTicks, null);
        builder.addDoubleProperty("Total Time", () -> motionProfile.totalTime(), null);
        builder.addBooleanProperty("masterRevSwitch", this::isBottomSwitchPressed, null);
        builder.addBooleanProperty("isAtGoal", this::isAtDesiredHeight, null);
        builder.addBooleanProperty("MotionMagicIsProfileFinished", () -> motionProfile.isFinished(timer.get()), null);
    }
}
