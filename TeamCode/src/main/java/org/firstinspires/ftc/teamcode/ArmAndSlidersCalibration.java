package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public class ArmAndSlidersCalibration extends LinearOpMode {
    private static final double ARM_KP = 0.1;
    private static final double ARM_KI = 0.01;
    private static final double ARM_KD = 0.01;

    private static final double SLIDE_KP = 0.1;
    private static final double SLIDE_KI = 0.01;
    private static final double SLIDE_KD = 0.01;
    private static final double MAX_ARM_ACCELERATION = 1.0;
    private static final double MAX_ARM_VELOCITY = 0.5;

    private static final double MAX_SLIDE_ACCELERATION = 0.5;
    private static final double MAX_SLIDE_VELOCITY = 0.2;

    public static double armSetpoint;
    public static double slideSetpoint;

    public double armErrorSum;
    public double slideErrorSum;

    public double armPrevError;
    public double slidePrevError;

    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor armMotor;
    public DcMotor slideMotor;

    @Override
    public void runOpMode() {
        // Inițializarea motoarelor și altor componente
//        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
//        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        initializeMotors();

        // Așteaptă pornirea robotului
        waitForStart();

        // Setarea poziției dorite pentru braț și glisiere
        armSetpoint = 1000; // Înlocuiește cu poziția dorită a brațului
        slideSetpoint = 500; // Înlocuiește cu poziția dorită a glisierelor

        while (opModeIsActive()) {
            updateRobot();
            telemetry.update();
        }
    }

    public void initializeMotors() {
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void updateRobot() {
        double currentArmPosition = getArmPosition();
        double currentSlidePosition = getSlidePosition();

        double slideElapsedTime = runtime.seconds();
        double slidePosition = motionProfilePosition(MAX_SLIDE_ACCELERATION, MAX_SLIDE_VELOCITY, slideSetpoint, slideElapsedTime);
        double slideError = slidePosition - currentSlidePosition;
        double slidePower = calculatePID(slideError, slidePrevError, slideErrorSum, SLIDE_KP, SLIDE_KI, SLIDE_KD);
        applySlidePower(slidePower);

        double armElapsedTime = runtime.seconds();
        double armPosition = motionProfilePosition(MAX_ARM_ACCELERATION, MAX_ARM_VELOCITY, armSetpoint, armElapsedTime);
        double armError = armPosition - currentArmPosition;
        double armPower = calculatePID(armError, armPrevError, armErrorSum, ARM_KP, ARM_KI, ARM_KD);
        applyArmPower(armPower);

        armPrevError = armError;
        armErrorSum += armError;

        slidePrevError = slideError;
        slideErrorSum += slideError;

        // Adaugă orice alte acțiuni necesare
        // Exemplu: actualizarea datelor de telemetrie
        telemetry.addData("Arm Position", currentArmPosition);
        telemetry.addData("Slide Position", currentSlidePosition);
    }


    private double calculatePID(double error, double prevError, double errorSum, double kp, double ki, double kd) {
        double pTerm = kp * error;
        double iTerm = ki * errorSum * getRuntime();
        double dTerm = kd * (error - prevError) / getRuntime();

        return pTerm + iTerm + dTerm;
    }

    private double motionProfilePosition(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {
        double accelerationTime = maxVelocity / maxAcceleration;

        double halfwayDistance = distance / 2.0;
        double accelerationDistance = 0.5 * maxAcceleration * accelerationTime * accelerationTime;

        if (accelerationDistance > halfwayDistance) {
            accelerationTime = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * accelerationTime * accelerationTime;

        maxVelocity = maxAcceleration * accelerationTime;

        double decelerationTime = accelerationTime;

        double cruiseDistance = distance - 2.0 * accelerationDistance;
        double cruiseTime = cruiseDistance / maxVelocity;
        double decelerationTimeTotal = accelerationTime + cruiseTime;

        if (elapsedTime > decelerationTimeTotal) {
            return distance;
        }

        if (elapsedTime < accelerationTime) {
            // Utilizăm ecuația cinematică pentru accelerație
            return 0.5 * maxAcceleration * elapsedTime * elapsedTime;
        } else if (elapsedTime < decelerationTimeTotal) {
            double accelerationDistancePart = 0.5 * maxAcceleration * accelerationTime * accelerationTime;
            double cruiseTimeCurrent = elapsedTime - accelerationTime;

            // Utilizăm ecuația cinematică pentru viteză constantă
            return accelerationDistancePart + maxVelocity * cruiseTimeCurrent;
        } else {
            double accelerationDistancePart = 0.5 * maxAcceleration * accelerationTime * accelerationTime;
            double cruiseDistancePart = maxVelocity * cruiseTime;
            double decelerationTimeCurrent = elapsedTime - decelerationTimeTotal;

            // Utilizăm ecuațiile cinematică pentru a calcula poziția dorită instantaneu
            double decelerationDistancePart = 0.5 * maxAcceleration * decelerationTime * decelerationTime;
            double decelerationPosition = accelerationDistancePart + cruiseDistancePart;

            if (decelerationTimeCurrent < decelerationTime) {
                // Decelerăm până la oprirea completă
                return decelerationPosition + maxVelocity * decelerationTimeCurrent
                        - 0.5 * maxAcceleration * decelerationTimeCurrent * decelerationTimeCurrent;
            } else {
                // Am ajuns la poziția inițială, ne oprim
                return distance;
            }
        }
    }


    private double getArmPosition() {
        return armMotor.getCurrentPosition();
    }

    private double getSlidePosition() {
        return slideMotor.getCurrentPosition();
    }

    private void applyArmPower(double power) {
        armMotor.setPower(power);
    }

    private void applySlidePower(double power) {
        slideMotor.setPower(power);
    }
}
