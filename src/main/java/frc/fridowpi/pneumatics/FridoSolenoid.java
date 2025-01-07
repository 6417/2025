package frc.fridowpi.pneumatics;

import frc.fridowpi.initializer.Initializer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Solenoid;

public class FridoSolenoid implements ISolenoid {

    private Solenoid proxy;
    private boolean initialized = false;
    private int channel;

    public FridoSolenoid(int channel) {
        this.channel = channel;
        Initializer.getInstance().addInitialisable(this);
        requires(PneumaticHandler.getInstance());
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
    public void init() {
        initialized = true;
        proxy = new Solenoid(PneumaticHandler.getInstance().getCompressorId(),
                PneumaticHandler.getInstance().getCompressorType(), channel);
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }

    @Override
    public void set(boolean direction) {
        proxy.set(direction);
    }

    @Override
    public int getChannel() {
        return this.channel;
    }

    @Override
    public void toggle() {
        proxy.set(!proxy.get());
    }

    @Override
    public boolean get() {
        return proxy.get();
    }
}
