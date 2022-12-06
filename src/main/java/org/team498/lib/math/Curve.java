package org.team498.lib.math;

import java.util.ArrayList;

public class Curve {
    private final double angle;
    double[] coefficients;
    public ArrayList<double[]> positions = new ArrayList<double[]>();
    private final double initialX;
    private final double target;
    private final boolean isTargetX;
    private boolean isIncreasing;
    double[] position;
    private int sample;
    public int length = 0;
    /**
     * Constructs a curve sampled from the coefficients between the initial values and target value
     * @param angle angle setpoint in degrees
     * @param coefficients double[] {c, b, a, ...} for polynomial: c + bx + ax^2 + ...
     * @param initialX initial x value
     * @param target end value (can be x or y)
     * @param isTargetX is target for the x or y value?
     */
    public Curve(double angle, double[] coefficients, double initialX, double target, boolean isTargetX) {
        this.angle = angle;
        this.coefficients = coefficients;
        this.initialX = initialX;
        this.target = target;
        this.isTargetX = isTargetX;
        this.isIncreasing = (isTargetX) ? initialX <= target : coefficients[0] <= target;
        generate(this);
    }
    public void add(Curve curveToAdd) {
        curveToAdd.generate(this);
    }
    public void generate(Curve curveBase) {
		double time = 0;
        sample = curveBase.length;
        position = calculatePosition(time);
        while (isNotFinished() && sample < 1500) {
			position = calculatePosition(time);
            curveBase.positions.add(sample, position);
            curveBase.length++;
            sample++;
			time += 10.0 / 1000.0;
        }
    }

    public double[] calculatePosition(double time) {
        time = (isIncreasing) ? time : -time;
        double y = 0;
        double x = time + initialX;
        double r = angle;
        for (int i = 0; i < coefficients.length; ++i) {
            y += Math.pow(time, i) * coefficients[i];
        }
        return new double[] {sample * 10.0 / 1000.0, x, y, r};
    }
    private boolean isNotFinished() {
        if (isIncreasing) {
            return (isTargetX) ? position[1] < this.target : position[2] < this.target;
        } else {
            return (isTargetX) ? position[1] > this.target : position[2] > this.target;
        }
    }
    public void concat(double angle, double[] coefficients, double target, boolean isTargetX) {
        double initialX = this.positions.get(this.length - 1)[1];
        coefficients[0] = this.positions.get(this.length - 1)[2];
        Curve curve2 = new Curve(angle, coefficients, initialX, target, isTargetX);
        this.add(curve2);
    }
}