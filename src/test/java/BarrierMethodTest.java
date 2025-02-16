import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.*;
import frc.fridowpi.utils.BarrierMethod;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

class BarrierMethodTest {
    @Test
    void testNewtonMinEx9_20() {
        BarrierMethod.Func<N2> f = x -> Math.exp(x.get(0) + 3 * x.get(1) - 0.1)
                + Math.exp(x.get(0) - 3 * x.get(1) - 0.1) + Math.exp(-x.get(0) - 0.1);
        BarrierMethod.Gradient<N2> gradf = x -> VecBuilder.fill(
                Math.exp(x.get(0) + 3 * x.get(1) - 0.1) + Math.exp(x.get(0) - 3 * x.get(1) - 0.1)
                        - Math.exp(-x.get(0) - 0.1),
                3 * Math.exp(x.get(0) + 3 * x.get(1) - 0.1) - 3 * Math.exp(x.get(0) - 3 * x.get(1) - 0.1));
        BarrierMethod.Hessian<N2> hessf = x -> MatBuilder.fill(Nat.N2(), Nat.N2(),
                Math.exp(x.get(0) + 3 * x.get(1) - 0.1) + Math.exp(x.get(0) - 3 * x.get(1) - 0.1)
                        + Math.exp(-x.get(0) - 0.1),
                3 * Math.exp(x.get(0) + 3 * x.get(1) - 0.1) - 3 * Math.exp(x.get(0) - 3 * x.get(1) - 0.1),
                3 * Math.exp(x.get(0) + 3 * x.get(1) - 0.1) - 3 * Math.exp(x.get(0) - 3 * x.get(1) - 0.1),
                9 * Math.exp(x.get(0) + 3 * x.get(1) - 0.1) + 9 * Math.exp(x.get(0) - 3 * x.get(1) - 0.1));

        Vector<N2> x = VecBuilder.fill(3.0, 1.0);
        x = BarrierMethod.newtonMinimize(f,
                gradf,
                hessf,
                (ignored) -> true,
                x,
                1e-14,
                0.1,
                0.7);

        Vector<N2> optimalX = VecBuilder.fill(-0.346573, 0.0);
        assertEquals(0.0, x.minus(optimalX).norm(), 1e-5 * optimalX.norm());
    }

    @Test
    void testBarrierLp() {
        BarrierMethod.Func<N2> obj = x -> x.get(0) + x.get(1);
        BarrierMethod.Gradient<N2> gradObj = x -> VecBuilder.fill(1.0, 1.0);
        BarrierMethod.Hessian<N2> hessObj = x -> MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 0.0, 0.0, 0.0);

        BarrierMethod.Func<N2> c1 = x -> -x.get(0) - 2 * x.get(1) + 3;
        BarrierMethod.Gradient<N2> gradC1 = x -> VecBuilder.fill(-1.0, -2.0);
        BarrierMethod.Hessian<N2> hessC1 = x -> MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 0.0, 0.0, 0.0);

        BarrierMethod.Func<N2> c2 = x -> x.get(0) - x.get(1) + 1;
        BarrierMethod.Gradient<N2> gradC2 = x -> VecBuilder.fill(1.0, -1.0);
        BarrierMethod.Hessian<N2> hessC2 = x -> MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 0.0, 0.0, 0.0);

        BarrierMethod.Func<N2> c3 = x -> -x.get(0);
        BarrierMethod.Gradient<N2> gradC3 = x -> VecBuilder.fill(-1.0, 0.0);
        BarrierMethod.Hessian<N2> hessC3 = x -> MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 0.0, 0.0, 0.0);

        BarrierMethod.Func<N2> c4 = x -> -x.get(1);
        BarrierMethod.Gradient<N2> gradC4 = x -> VecBuilder.fill(0.0, -1.0);
        BarrierMethod.Hessian<N2> hessC4 = x -> MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 0.0, 0.0, 0.0);

        BarrierMethod<N2> solver = new BarrierMethod<N2>(obj, gradObj, hessObj);
        solver.addConstr(c1, gradC1, hessC1);
        solver.addConstr(c2, gradC2, hessC2);
        solver.addConstr(c3, gradC3, hessC3);
        solver.addConstr(c4, gradC4, hessC4);

        Vector<N2> x = VecBuilder.fill(1.0, 2.5);
        assertTrue(solver.isFeasible(x));
        x = solver.solve(x);

        System.out.println("sol: " + x);
        System.out.println("sol value: " + obj.apply(x));

        Vector<N2> optimalX = VecBuilder.fill(0.0, 1.5);

        assertEquals(0.0, x.minus(optimalX).norm(), 1e-5 * optimalX.norm());
    }

    <N extends Num> BarrierMethod.Func<N> makeQcqpFunc(double r, Matrix<N, N> P, Vector<N> q) {
        return x -> 0.5 * x.transpose().times(P.times(x)).get(0, 0) + q.dot(x) + r;
    }

    <N extends Num> BarrierMethod.Gradient<N> gradientFunc(double r,
            Matrix<N, N> P, Vector<N> q) {
        return x -> new Vector<>(P.times(x).plus(q));
    }

    <N extends Num> BarrierMethod.Hessian<N> hessianFunc(double r,
            Matrix<N, N> P, Vector<N> q) {
        return x -> P;
    }

    <N extends Num> void genericQcqpTest(
            Matrix<N, N> P, Vector<N> q, double r,
            List<Matrix<N, N>> P1, List<Vector<N>> q1, List<Double> r1,
            Vector<N> initialGuess, Vector<N> optimalX, double eps) {
        BarrierMethod.Func<N> obj = makeQcqpFunc(r, P, q);
        BarrierMethod.Gradient<N> gradObj = gradientFunc(r, P, q);
        BarrierMethod.Hessian<N> hessObj = hessianFunc(r, P, q);

        BarrierMethod<N> solver = new BarrierMethod<N>(obj, gradObj, hessObj);

        for (int i = 0; i < P1.size(); i++) {
            BarrierMethod.Func<N> c1 = makeQcqpFunc(r1.get(i), P1.get(i), q1.get(i));
            BarrierMethod.Gradient<N> gradC1 = gradientFunc(r1.get(i), P1.get(i), q1.get(i));
            BarrierMethod.Hessian<N> hessC1 = hessianFunc(r1.get(i), P1.get(i), q1.get(i));
            solver.addConstr(c1, gradC1, hessC1);
        }

        Vector<N> x = initialGuess;
        assertTrue(solver.isFeasible(x));
        x = solver.solve(x);

        assertEquals(0.0, x.minus(optimalX).norm(), eps * optimalX.norm());

        System.out.println("sol: " + x);
        System.out.println("sol value: " + obj.apply(x));
    }

    @Test
    void testBarrierQcqpSimple() {
        Matrix<N2, N2> P = MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 0.0, 0.0, 0.0);
        Vector<N2> q = VecBuilder.fill(1.0, 0.0);
        double r = 0.0;

        Matrix<N2, N2> P1 = MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0);
        Vector<N2> q1 = VecBuilder.fill(0.0, 0.0);
        double r1 = -1.0;

        Vector<N2> initialGuess = VecBuilder.fill(0.5, 0.5);
        Vector<N2> optimalX = VecBuilder.fill(-Math.sqrt(2), 0.0);

        genericQcqpTest(P, q, r, List.of(P1), List.of(q1), List.of(r1), initialGuess, optimalX, 1e-5);
    }

    @Test
    void testBarrierRandomQcqp() {
        Matrix<N2, N2> P = MatBuilder.fill(Nat.N2(), Nat.N2(), 0.49124, 0.281095, 0.281095, 0.444194);
        Vector<N2> q = VecBuilder.fill(0.659969, 0.653366);
        double r = 2.0;

        Matrix<N2, N2> P1 = MatBuilder.fill(Nat.N2(), Nat.N2(), 0.748865, 0.597822, 0.597822, 0.51972);
        Vector<N2> q1 = VecBuilder.fill(0.703999, 0.0874274);
        double r1 = 0.48269;

        Vector<N2> initialGuess = VecBuilder.fill(-2.0, 1.0);
        Vector<N2> optimalX = VecBuilder.fill(-1.3639, 0.342858);

        genericQcqpTest(P, q, r, List.of(P1), List.of(q1), List.of(r1), initialGuess, optimalX, 1e-3);
    }

    @Test
    void testInteriorOptimum() {
        Matrix<N2, N2> P = MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.2, 0.2, 1.5);
        Vector<N2> q = VecBuilder.fill(0.0, 0.0);
        double r = -0.5;

        BarrierMethod.Func<N2> obj = makeQcqpFunc(r, P, q);
        BarrierMethod.Gradient<N2> gradObj = gradientFunc(r, P, q);
        BarrierMethod.Hessian<N2> hessObj = hessianFunc(r, P, q);

        Matrix<N2, N2> P1 = MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.3, 0.3, 1.0);
        Vector<N2> q1 = VecBuilder.fill(0.5, 0.5);
        double r1 = -1.0;

        BarrierMethod.Func<N2> c1 = makeQcqpFunc(r1, P1, q1);
        BarrierMethod.Gradient<N2> gradC1 = gradientFunc(r1, P1, q1);
        BarrierMethod.Hessian<N2> hessC1 = hessianFunc(r1, P1, q1);

        BarrierMethod<N2> solver = new BarrierMethod<N2>(obj, gradObj, hessObj);
        solver.addConstr(c1, gradC1, hessC1);

        Vector<N2> x = VecBuilder.fill(0.5, 0.5);
        assertTrue(solver.isFeasible(x));
        x = solver.solve(x);

        Vector<N2> optimalX = VecBuilder.fill(0.0, 0.0);
        assertEquals(0.0, x.minus(optimalX).norm(), 1e-4);
    }
}
