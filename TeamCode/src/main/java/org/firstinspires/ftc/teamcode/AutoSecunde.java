//package org.firstinspires.ftc.teamcode;
//
//import static java.lang.Thread.sleep;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//@TeleOp(name = "AUTONOMSECUNDE")
//public class AutoSecunde extends OpMode {
//
//    private DcMotor leftFront,leftRear,rightRear,rightFront;
//
//    @Override
//    public void init() {
//
//        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront_OdometryLeft");
//        leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
//        rightRear = hardwareMap.get(DcMotorEx.class, "RightBack_OdometryFront");
//        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront_OdometryRight");
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        leftRear.setDirection(DcMotor.Direction.REVERSE);
//        rightRear.setDirection(DcMotor.Direction.FORWARD);
//
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void loop() {
//        Forward(0.5, 500);
//        sleep(1000);
//        StrafeDreapta(0.5, 2000);
//
//    }
//    public void Forward(double power, long milliseconds) {
//        // Setează puterea motoarelor
//        leftFront.setPower(power);
//        rightFront.setPower(power);
//        leftRear.setPower(power);
//        rightRear.setPower(power);
//
//        // Așteaptă pentru numărul specificat de milisecunde
//        sleep(milliseconds);
//
//        // Oprește motoarele
//        stopMotors();
//    }
//    public void StrafeDreapta(double power, long milliseconds) {
//        // Setează puterea motoarelor
//        leftFront.setPower(power);
//        rightFront.setPower(-power);
//        leftRear.setPower(-power);
//        rightRear.setPower(power);
//
//        // Așteaptă pentru numărul specificat de milisecunde
//        sleep(milliseconds);
//
//        // Oprește motoarele
//        stopMotors();
//    }
//    public void stopMotors() {
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//        leftRear.setPower(0);
//        rightRear.setPower(0);
//    }
//}
