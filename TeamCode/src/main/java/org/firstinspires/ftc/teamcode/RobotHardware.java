package org.firstinspires.ftc.teamcode;

import static java.lang.Runtime.getRuntime;

import org.firstinspires.ftc.teamcode.BratMotionProfile;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.ArmAndSlidersCalibration;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import kotlin.jvm.functions.Function2;

@Config
public class RobotHardware {
    ElapsedTime elapsedTime = new ElapsedTime();

    public DcMotorEx
            leftFront, leftRear,
            rightRear, rightFront,
            Lift,
            BratDreapta, BratStanga;
    public Servo
            GhearaStanga, GhearaDreapta,
            GhearaInclinatie, Drona;

    double closeClaw = 0.5, openClaw = 0;
    double InclineClawRetract = -1;
    double InclineClawParallel = 0.8;
    boolean buttonIsPressed = false, toggleClaw = false, button2IsPressed = false, toggleInclineClaw = false, toggleDrone = false, button3IsPressed = false;
    public double pid;
    boolean manualControl = false;
    PIDController pidController = new PIDController(0, 0, 0);

    ArmAndSlidersCalibration ArmAndSlidersCalibration;

    public static double kpLIFT = 0, kiLIFT = 0, kdLIFT = 0, ffLIFT = 0;
    public static int liftTarget = 0;
//    public static double kpBRAT = 0.1, kiBRAT = 0, kdBRAT = 0, ffBRAT = 0.3, kCos = 0.0, kgBrat = 0.0;
    private final double ZERO_OFFSET = 0;
    public static double ticks_in_degrees = 288 / 180.0;
//    public static int BratTarget = 0;

    int high = 0, medium = 0, low = 0;
    double InitDrone = -1;
    int LaunchPos = 1;

    private double maxAcceleration = 0.0; // Example value, adjust as needed
    private double maxVelocity = 0.0; // Example value, adjust as needed
    private double distance = 0.0; // Example value, adjust as needed
    private double bratKp = 0.0;

    public IMU imu;


//    public double deceleration_time = 0.0;
//    public double acceleration_dt = 0.0, deceleration_dt = 0.0,
//            cruise_dt = 0.0, entire_dt = 0.0, cruise_current_dt = 0.0;
//    public double halfway_distance = 0.0,
//            acceleration_distance = 0.0, cruise_distance = 0.0;

    public RobotHardware(HardwareMap hardwareMap) {

        //region MotoareDrive
        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront_OdometryLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "RightBack_OdometryFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront_OdometryRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        ArmAndSlidersCalibration.armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        ArmAndSlidersCalibration.slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        ArmAndSlidersCalibration.initializeMotors();

        //endregion

        //region MotoareLift
//        Lift = hardwareMap.get(DcMotorEx.class, "Brat");
//
//        BratStanga = hardwareMap.get(DcMotorEx.class, "BratStanga");
//        BratDreapta = hardwareMap.get(DcMotorEx.class, "BratDreapta");
//
//        Lift.setDirection(DcMotorEx.Direction.FORWARD);

//        BratStanga.setDirection(DcMotor.Direction.FORWARD);
//        BratDreapta.setDirection(DcMotor.Direction.FORWARD);

//        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        BratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        BratStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BratDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //endregion

        //region ServoGheara

        GhearaStanga = hardwareMap.get(Servo.class, "GhearaStanga");
        GhearaDreapta = hardwareMap.get(Servo.class, "GhearaDreapta");

        Drona = hardwareMap.get(Servo.class, "Drona");

        GhearaStanga.setDirection(Servo.Direction.FORWARD);
        GhearaDreapta.setDirection(Servo.Direction.REVERSE);

        Drona.setDirection(Servo.Direction.FORWARD);

        GhearaInclinatie = hardwareMap.get(Servo.class, "GhearaInclinatie");

        GhearaInclinatie.setDirection(Servo.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        elapsedTime.reset();

        //endregion

        //LynxBulkCachingMode
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    public void RetractClaw() {
        InclineClawState(InclineClawRetract);
    }

    public void ParallelClaw() {
        InclineClawState(InclineClawParallel);}


    public void ClawStateIndivi(double pos,boolean stg) {
        if(stg)
            GhearaStanga.setPosition(pos);
        else
            GhearaDreapta.setPosition(pos);}


    public void ClawState(double pos) {
        GhearaStanga.setPosition(pos);
        GhearaDreapta.setPosition(pos);}

    public void InitDrone() {
        Drona.setPosition(InitDrone);
    }

    public void LaunchDrone() {
        Drona.setPosition(LaunchPos);
    }

    public void InclineClawState(double pos) {
        GhearaInclinatie.setPosition(pos);
    }

   public void OpenClaw() {
      ClawState(openClaw);
   }

//    public void Drone(Gamepad gamepad) {
//        if(gamepad.y)
//            LaunchDrone();
//    }

    public void DriveMovement(Gamepad gamepad) {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        if (!gamepad.left_bumper) {
            x /= 2;
            y /= 2;
        }
        if (!gamepad.right_bumper) {
            rx /= 2;
        }

        if (gamepad.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double frontLeftPower = (rotY + rotX + rx);
        double backLeftPower = (rotY - rotX + rx);
        double frontRightPower = (rotY - rotX - rx);
        double backRightPower = (rotY + rotX - rx);

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    public void LiftPID(Gamepad gamepad) {

        double ManualLiftPower = (gamepad.left_trigger-gamepad.right_trigger+ffLIFT)*0.5;
        if(gamepad.left_trigger > 0.1 || gamepad.right_trigger > 0.1)
            manualControl = true;
        if(gamepad.left_trigger > 0.9 || gamepad.right_trigger > 0.9)
            ManualLiftPower = (gamepad.left_trigger-gamepad.right_trigger+ffLIFT)*0.7;

        pidController.setPID(kpLIFT, kiLIFT, kdLIFT);
        int LiftPos = Lift.getCurrentPosition();
        double pid = pidController.calculate(LiftPos, liftTarget);
        double pidPowerLIFT = pid + ffLIFT;

        if (manualControl) {
            Lift.setPower(ManualLiftPower);
//            LiftDreapta.setPower(manualPower);
//            LiftStanga.setPower(manualPower);
        } else {
            Lift.setPower(pidPowerLIFT);
//            LiftDreapta.setPower(pidPower);
//            LiftStanga.setPower(pidPower);
        }
    }


    public void BratPID(Gamepad gamepad) {
//
////        double manualArmPower = (gamepad.left_stick_y - gamepad.right_stick_y + ffBRAT) * 0.5;
////
////        if (gamepad.left_stick_y > 0.1 || gamepad.right_stick_y > 0.1)
////            manualControl = true;
////        if (gamepad.left_stick_y > 0.9 || gamepad.right_stick_y > 0.9)
////            manualArmPower = (gamepad.left_stick_y - gamepad.right_stick_y + ffBRAT) * 0.7;
//
////       double manualArmPower = (gamepad.left_trigger - gamepad.right_trigger + ffBRAT) * 0.2;
////
////        if (gamepad.left_trigger > 0.1 || gamepad.right_trigger > 0.1)
////            manualControl = true;
////        if (gamepad.left_trigger > 0.9 || gamepad.right_trigger > 0.9)
////            manualArmPower = (gamepad.left_trigger - gamepad.right_trigger + ffBRAT) * 0.5;
//
//        /*
//            double currPosInTicks = armMotor.getCurrentPosition();
//            double ff = ffBRAT * Math.sin(Math.toRadians(ticksToRealWorldDegrees(currPosInTicks)));
//            public double ticksToRealWorldDegrees(double ticks)
//            {
//                return ticks / ticksPerDegree + zeroOffset;
//            }
//            double power = pidOutput + ff;
//            power = Range.clip(power, -powerLimit, powerLimit);
//         */
//
//        pidController.setPID(kpBRAT, kiBRAT, kdBRAT);
//
//        int armPos = BratStanga.getCurrentPosition();
//        double pid = pidController.calculate(armPos, BratTarget);
//        double pidPowerBrat = pid + ffBRAT;
//
////        double instantTargetPosition = motion_profile(max_acceleration, max_velocity, distance, elapsed_time);
////
////        double motorPower = (instantTargetPosition - BratStanga.getCurrentPosition()) * kpBRAT;
//
//        BratDreapta.setPower(pidPowerBrat);
//        BratStanga.setPower(pidPowerBrat);
//
//        if(gamepad.dpad_up)
//            BratTarget = BratTarget + 1;
//        if(gamepad.dpad_down)
//            BratTarget = BratTarget - 1;
        double startTime = elapsedTime.seconds();
        double targetposition = BratMotionProfile.calculatePosition(maxAcceleration, maxVelocity, distance, startTime);
        double BratPower = (targetposition - Lift.getCurrentPosition()) * bratKp;

        Lift.setPower(BratPower);

    }

    public void DroneManager(Gamepad gamepad) {
        if (gamepad.y && !button3IsPressed) {
            if (toggleDrone)
                Drona.setPosition(InitDrone);
            else
                Drona.setPosition(LaunchPos);
            button3IsPressed = true;
            toggleDrone = !toggleDrone;
        } else if (!gamepad.y)
            button3IsPressed = false;
    }

    public void InclineClawManager(Gamepad gamepad) {

        if (gamepad.right_bumper && !button2IsPressed) {
            if (toggleInclineClaw)
                InclineClawState(InclineClawRetract);
            else
                InclineClawState(InclineClawParallel);
            button2IsPressed = true;
            toggleInclineClaw = !toggleInclineClaw;
        } else if (!gamepad.right_bumper)
            button2IsPressed = false;

    }


//    public void InitBratLift() {
//        ServoStanga.setPosition(0);
//        ServoDreapta.setPosition(1);
//    }
//
//    public void ArmPos(Gamepad gamepad)  {
//        if(gamepad.a) {
//            ServoStanga.setPosition(0);
//            ServoDreapta.setPosition(1);
//        }
//        if(gamepad.b) {
//            ServoStanga.setPosition(1);
//            ServoDreapta.setPosition(0);
//        }
//        if(gamepad.y) {
//            ServoStanga.setPosition(0.8);
//            ServoDreapta.setPosition(0.2);
//        }
//    }

    public void setLiftTarget(int pos) {
        liftTarget = pos;
    }

//    public void setBratTarget(int pos) {
//        BratTarget = pos;
//    }

    public void ClawManager(Gamepad gamepad) {

        if (gamepad.x && !buttonIsPressed) {
            if (toggleClaw)
                ClawState(closeClaw);
            else
                ClawState(openClaw);
            buttonIsPressed = true;
            toggleClaw = !toggleClaw;
        } else if (!gamepad.x)
            buttonIsPressed = false;

    }
    public void openClawStanga(){
        GhearaStanga.setPosition(openClaw);
    }

    public void openClawDreapta(){
        GhearaDreapta.setPosition(openClaw);
    }

//    public void inclinaGheara(){
//        GhearaInclinatie.setPosition(InclineClawParallel);
//    }
//
//    public void retractGheara (){
//        GhearaInclinatie.setPosition(0);
//    }
    public void ClawStanga  (Gamepad gamepad) {

        if (gamepad.x && !buttonIsPressed) {
            if (toggleClaw)
                ClawStateIndivi(closeClaw, true);
            else
                ClawStateIndivi(openClaw, true);
            buttonIsPressed = true;
            toggleClaw = !toggleClaw;
        } else if (!gamepad.x)
            buttonIsPressed = false;

    }

    public void ClawDreapta(Gamepad gamepad) {

        if (gamepad.b && !buttonIsPressed) {
            if (toggleClaw)
                ClawStateIndivi(closeClaw,false);
            else
                ClawStateIndivi(openClaw,false);
            buttonIsPressed = true;
            toggleClaw = !toggleClaw;
        } else if (!gamepad.b)
            buttonIsPressed = false;

    }

    public int getLiftLeftPosition() {
        return Lift.getCurrentPosition();
//        return LiftStanga.getCurrentPosition();
    }

    public int getArmPosition() {
        return BratStanga.getCurrentPosition();
    }

//    public int getLiftRightPosition()
//    {
//        return Lift.getCurrentPosition();
////        return LiftDreapta.getCurrentPosition();
//    }

//    public int getArmTarget() {
//        return BratTarget;
//    }

        public int getLiftTarget ()
        {
            return liftTarget;
        }
        public double getArmPower() {
            return BratStanga.getPower();
        }

        public double getLiftPower ()
        {
            return Lift.getPower();
//        return LiftDreapta.getPower();
        }

//    public void RetractClaw() {
//        InclineClawState(InclineClawRetract);
//    }
//
//    public void ParallelClaw() {
//        InclineClawState(InclineClawParallel);}

//    public void InclineClawState(double pos) {
//        GhearaInclinatie.setPosition(pos);
//    }

    public void IndividualClawRightState(double pos) {
        GhearaDreapta.setPosition(pos);
    }

    public void ClawRightOpen() {
        IndividualClawRightState(closeClaw);
    }

    public void ClawRightClose() {
        IndividualClawRightState(openClaw);
    }

    public void IndividualClawLeftState(double pos) {
        GhearaStanga.setPosition(pos);
    }

    public void ClawLeftOpen() {
        IndividualClawLeftState(closeClaw);
    }

    public void ClawLeftClose() {
        IndividualClawLeftState(openClaw);
    }
    }

