package org.firstinspires.ftc.teamcode;
public class BratMotionProfile {
    public static double calculatePosition(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {
        // Calculate the time it takes to accelerate to max velocity
        double accelerationTime = maxVelocity / maxAcceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfwayDistance = distance / 2;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationTime = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);

        // Recalculate max velocity based on the time we have to accelerate and decelerate
        maxVelocity = maxAcceleration * accelerationTime;

        // We decelerate at the same rate as we accelerate
        double decelerationTime = accelerationTime;

        // Calculate the time that we're at max velocity
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseTime = cruiseDistance / maxVelocity;
        double decelerationTimeTotal = accelerationTime + cruiseTime;

        // Check if we're still in the motion profile
        double entireTime = accelerationTime + cruiseTime + decelerationTime;
        if (elapsedTime > entireTime) {
            return distance;
        }

        // If we're accelerating
        if (elapsedTime < accelerationTime) {
            // Use the kinematic equation for acceleration
            return 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        }

        // If we're cruising
        else if (elapsedTime < decelerationTimeTotal) {
            double cruiseCurrentTime = elapsedTime - accelerationTime;

            // Use the kinematic equation for constant velocity
            return accelerationDistance + maxVelocity * cruiseCurrentTime;
        }

        // If we're decelerating
        else {
            double cruiseDistanceTotal = maxVelocity * cruiseTime;
            double decelerationTimeElapsed = elapsedTime - decelerationTimeTotal;

            // Use the kinematic equations to calculate the instantaneous desired position
            return accelerationDistance + cruiseDistanceTotal + maxVelocity * decelerationTimeElapsed
                    - 0.5 * maxAcceleration * Math.pow(decelerationTimeElapsed, 2);
        }
    }
}
