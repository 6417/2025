package frc.fridowpi.pneumatics;

import frc.fridowpi.initializer.Initializer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class FridoDoubleSolenoid implements IDoubleSolenoid {
    private boolean initialized = false;
    private final int lowerId;
    private final int higherId;
    private DoubleSolenoid proxy;

    public FridoDoubleSolenoid(int lowerId, int higherId) {
        this.lowerId = lowerId;
        this.higherId = higherId;
        Initializer.getInstance().addInitialisable(this);
    }

    @Override
    public void init() {
        initialized = true;
        proxy = new DoubleSolenoid(PneumaticHandler.getInstance().getCompressorId(),
                PneumaticHandler.getInstance().getCompressorType(), lowerId, higherId);
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }

    @Override
    public void close() throws Exception {
        proxy.close();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        proxy.initSendable(builder);
    }

    @Override
    public void set(Value value) {
        proxy.set(value);
    }

    @Override
    public Value get() {
        return proxy.get();
    }

    @Override
    public void toggle() {
        proxy.toggle();
    }

    @Override
    public int getFwdChannel() {
        return proxy.getFwdChannel();
    }

    @Override
    public int getRevChannel() {
        return proxy.getRevChannel();
    }

    @Override
    public boolean isFwdSolenoidDisabled() {
        return proxy.isFwdSolenoidDisabled();
    }

    @Override
    public boolean isRevSolenoidDisabled() {
        return proxy.isRevSolenoidDisabled();
    }

}
