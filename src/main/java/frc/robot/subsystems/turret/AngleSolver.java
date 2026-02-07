package frc.robot.subsystems.turret;

public class AngleSolver {
    public double solvePitch(double distance) {
        double theta = Math.toRadians(88); 
        double g = 9.8;
        double d = distance;
        double v = 9;
        double h = 1.8;
        for (int i = 0; i < 100; i++) {
            double func = d * Math.tan(theta) - (g * d * d ) / (2 * (v * Math.cos(theta)) * (v * Math.cos(theta))) - h;
            System.out.println(theta);
            if (Math.abs(func) < 0.001) {
                return theta;
            }
            double derivative = (d * (1 / Math.cos(theta)) * (1 / Math.cos(theta)) - (  4 * g * d * d * (v * Math.cos(theta)) * v * Math.sin(theta)) / ((2 * (v * Math.cos(theta)) * (v * Math.cos(theta))) * (2 * (v * Math.cos(theta)) * (v * Math.cos(theta)))) );
            double step = func / derivative;
            theta = theta - step;
        }
        return theta;
    }

    public static void main(String[] args) {
        AngleSolver angleSolver = new AngleSolver();
        double distance = 6.13;
        double theta = angleSolver.solvePitch(distance);
        System.out.println(theta);
    }
}