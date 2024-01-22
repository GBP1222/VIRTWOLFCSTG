//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class MotorControl {
//
//    private DcMotor motorStanga;
//    private DcMotor motorDreapta;
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    public MotorControl(DcMotor motorStanga, DcMotor motorDreapta) {
//        this.motorStanga = motorStanga;
//        this.motorDreapta = motorDreapta;
//
//        motorDreapta.setDirection(DcMotor.Direction.REVERSE);
//    }
//
//    public void moveMotorsForSeconds(double power, long milliseconds) {
//        motorStanga.setPower(power);
//        motorDreapta.setPower(power);
//
//        sleep(milliseconds);
//
//        stopMotors();
//    }
//
//    public void stopMotors() {
//        motorStanga.setPower(0);
//        motorDreapta.setPower(0);
//    }
//
//    private void sleep(long milliseconds) {
//        try {
//            Thread.sleep(milliseconds);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//    }
//}
