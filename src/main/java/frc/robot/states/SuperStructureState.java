package frc.robot.states;

import edu.wpi.first.units.measure.Angle;

public class SuperStructureState{
    private double angle; // in Degrees or Radians
    private double height; // in Meters

    public SuperStructureState(double angle, double height) {
        this.angle = angle;
        this.height = height;
    }

    public double getAngle() {
        return angle;
    }

    public double getHeight() {
        return height;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public void setHeight(double height) {
        this.height = height;
    }
}