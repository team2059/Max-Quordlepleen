package frc.robot.utils;

public class ShooterState {

    double predictedAngle;
    double predictedVelocity;

    public ShooterState(double predictedVelocity, double predictedAngle) {
        this.predictedAngle = predictedAngle;
        this.predictedVelocity = predictedVelocity;
    }

}