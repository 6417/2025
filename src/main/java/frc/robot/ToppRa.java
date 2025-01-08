package frc.robot;

import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.optim.linear.LinearConstraint;

public class ToppRa {
    public interface Curve {
        RealVector calc_pos(double s);
    } 

    public class PositionConstraint 
    {
        LinearConstraint l;
    }

    public class VelocityConstraint 
    {

    }

    public ToppRa(Curve curve, double sEnd) {
    }
}
