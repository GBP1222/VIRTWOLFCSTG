package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class ArmAndSlidersCalibration extends LinearOpMode {
    public static double ARM_KP = 0.1;
    public static double ARM_KI = 0.01;
    public static double ARM_KD = 0.01;

    public static double SLIDE_KP = 0.1;
    public static double SLIDE_KI = 0.01;
    public static double SLIDE_KD = 0.01;
    public static double MAX_ARM_ACCELERATION = 1.0;
    public static double MAX_ARM_VELOCITY = 0.5;

    public static double MAX_SLIDE_ACCELERATION = 0.5;
    public static double MAX_SLIDE_VELOCITY = 0.2;

    public static double armSetpoint;
    public static double slideSetpoint;

    public double armErrorSum;
    public double slideErrorSum;

    public double armPrevError;
    public double slidePrevError;

    public DcMotor armMotor;
    public DcMotor slideMotor;

    public double initialArmPosition;
    public double initialSlidePosition;

    public ArmAndSlidersCalibration(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void runOpMode() {
        // Inițializarea motoarelor și altor componente
        armMotor = hardwareMap.get(DcMotor.class, "Brat");
        slideMotor = hardwareMap.get(DcMotor.class, "Lift");

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        initializeMotors();
//
//        // Resetarea encoderelor la începutul teleop-ului
//        resetEncoders();

        // Așteaptă pornirea robotului
        waitForStart();

        // Salvează poziția inițială a encoderelor
        initialArmPosition = getArmPosition();
        initialSlidePosition = getSlidePosition();

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

    public void resetEncoders() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateRobot() {
        if(opModeIsActive()) {
            double currentArmPosition = getArmPosition() + initialArmPosition; //-
            double currentSlidePosition = getSlidePosition() + initialSlidePosition; //-

            double slidePosition = motionProfilePosition(MAX_SLIDE_ACCELERATION, MAX_SLIDE_VELOCITY, slideSetpoint, currentSlidePosition);
            double slideError = slidePosition - currentSlidePosition;
            double slidePower = calculatePID(slideError, slidePrevError, slideErrorSum, SLIDE_KP, SLIDE_KI, SLIDE_KD);
            applySlidePower(slidePower);

            double armPosition = motionProfilePosition(MAX_ARM_ACCELERATION, MAX_ARM_VELOCITY, armSetpoint, currentArmPosition);
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
        else
        {
            applyArmPower(0.0);
            applySlidePower(0.0);
        }
    }

    public double calculatePID(double error, double prevError, double errorSum, double kp, double ki, double kd) {
        double pTerm = kp * error;
        double iTerm = ki * errorSum;
        double dTerm = kd * (error - prevError);

        return pTerm + iTerm + dTerm;
    }

    public double motionProfilePosition(double maxAcceleration, double maxVelocity, double setpoint, double currentPosition) {
        double accelerationTime = maxVelocity / maxAcceleration;

        double halfwayDistance = setpoint / 2.0;
        double accelerationDistance = 0.5 * maxAcceleration * accelerationTime * accelerationTime;

        if (accelerationDistance > halfwayDistance) {
            accelerationTime = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * accelerationTime * accelerationTime;

        maxVelocity = maxAcceleration * accelerationTime;

        double decelerationTime = accelerationTime;

        double cruiseDistance = setpoint - 2.0 * accelerationDistance;
        double cruiseTime = cruiseDistance / maxVelocity;

        double totalTime = accelerationTime + cruiseTime + decelerationTime;

        currentPosition = Math.max(0, Math.min(currentPosition, setpoint));

        if (currentPosition > setpoint) {
            return setpoint;
        }

        if (currentPosition < accelerationDistance) {
            return 0.5 * maxAcceleration * Math.pow(currentPosition / maxAcceleration, 2);
        } else if (currentPosition < accelerationDistance + cruiseDistance) {
            double accelerationDistancePart = 0.5 * maxAcceleration * accelerationTime * accelerationTime;
            double cruiseTimeCurrent = (currentPosition - accelerationDistance) / maxVelocity;

            return accelerationDistancePart + maxVelocity * cruiseTimeCurrent;
        } else {
            double accelerationDistancePart = 0.5 * maxAcceleration * accelerationTime * accelerationTime;
            double cruiseDistancePart = maxVelocity * cruiseTime;

            double decelerationTimeCurrent = Math.sqrt((currentPosition - accelerationDistancePart - cruiseDistancePart) * 2 / maxAcceleration);
            double decelerationPosition = accelerationDistancePart + cruiseDistancePart;

            if (decelerationTimeCurrent < decelerationTime) {
                return decelerationPosition + maxVelocity * decelerationTimeCurrent
                        - 0.5 * maxAcceleration * Math.pow(decelerationTimeCurrent, 2);
            } else {
                return setpoint;
            }
        }
    }

//    public double getArmPosition() {
//        return armMotor.getCurrentPosition();
//    }
//
//    public double getSlidePosition() {
//        return slideMotor.getCurrentPosition();
//    }

    public double getArmPosition() {
        if (armMotor != null) {
            return armMotor.getCurrentPosition();
        } else {
            return 0.0;
        }
    }

    public double getSlidePosition() {
        if (slideMotor != null) {
            return slideMotor.getCurrentPosition();
        } else {
            return 0.0;
        }
    }

    public void applyArmPower(double power) {
        if (armMotor != null) {
            armMotor.setPower(power);
        } else {
            telemetry.update();
        }
    }

    public void applySlidePower(double power) {
        if (slideMotor != null) {
            slideMotor.setPower(power);
        } else {
            telemetry.update();
        }
    }


    public void setSlideSetpoint(double position) {
        slideSetpoint = position;
    }

    public void setArmSetpoint(double position) {
        armSetpoint = position;
    }


}
