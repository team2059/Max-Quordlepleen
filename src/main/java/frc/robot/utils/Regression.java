package frc.robot.utils;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class Regression {

    // Sample data points
    private static double[] distances = new double[] { 1, 6, 7, 9, 12, 20 };
    private static double[] velocities = new double[] { 2, 8, 6, 10, 14, 41 };

    // Method to calculate desired velocity based on distance
    public static double calculateDesiredVelocity(double distance) {
        SplineInterpolator interpolator = new SplineInterpolator();
        PolynomialSplineFunction velocityFunction = interpolator.interpolate(distances, velocities);

        // Ensure the distance is within the bounds of the original data
        if (distance >= distances[0] && distance <= distances[distances.length - 1]) {
            return velocityFunction.value(distance);
        } else {
            // Handle the case where the distance is out of bounds
            System.out.println("Distance is out of the valid range.");
            return 0;
        }

    }

    public static void main(String[] args) {

        System.out.println(calculateDesiredVelocity(3.1415));

    }

}
