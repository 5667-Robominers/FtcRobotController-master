package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Thread.sleep;

public class hardwareDevices {

    public DcMotorEx front_left, front_right, back_left, back_right;
    public Servo servo;
    public BNO055IMU imu;

    Constants constants = new Constants();

    private ElapsedTime runtime = new ElapsedTime();

    private double globalAngle;

    LinearOpMode op;

    private double angleToPreferred;

    HardwareMap hwMap = null;

    public hardwareDevices(LinearOpMode op) {
        this.op = op;
    }

    public void init(HardwareMap ahwMap) {


        hwMap = ahwMap;

        // Define and Initialize Motors
        front_left = hwMap.get(DcMotorEx.class, "front_left");
        front_right = hwMap.get(DcMotorEx.class, "front_right");
        back_left = hwMap.get(DcMotorEx.class, "back_left");
        back_right = hwMap.get(DcMotorEx.class, "back_right");


        front_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        front_left.setVelocity(0);
        front_right.setVelocity(0);
        back_left.setVelocity(0);
        back_right.setVelocity(0);

    }


    public void stopDriveChain() {
        front_left.setVelocity(0);
        front_right.setVelocity(0);
        back_left.setVelocity(0);
        back_right.setVelocity(0);
    }
    //35kg servo


    public void turnTo(double angle) {

        float currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angleToDesired = angle - currentAngle;

        while (Math.abs(angleToDesired) > 1 && op.opModeIsActive() && !op.isStopRequested()) {

            angleToDesired = angle - currentAngle;

//            turn(1 * Math.abs(angleToDesired / 30));
            if (angleToDesired > 0) {
                turn(-1 * Math.abs(angleToDesired / 30));
            } else if (angleToDesired < 0){
                turn(1 * Math.abs(angleToDesired / 30));
            }

            currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            op.telemetry.addLine()
                    .addData("imu", currentAngle);
            op.telemetry.update();

        }
        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
        sleep(400);
        currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        angleToDesired = angle - currentAngle;

        while (Math.abs(angleToDesired) > 1 && op.opModeIsActive() && !op.isStopRequested()) {


            angleToDesired = angle - currentAngle;

//            turn(1 * Math.abs(angleToDesired / 30));
            if (angleToDesired > 0) {
                turn(-0.6 * Math.abs(angleToDesired / 30));
            } else if (angleToDesired < 0){
                turn(0.6 * Math.abs(angleToDesired / 30));
            }

            currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            op.telemetry.addLine()
                    .addData("imu", currentAngle);
            op.telemetry.update();

        }
        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
        sleep(400);
    }

    //Positive doubles cause clockwise rotation
    //Negative doubles cause counterclockwise rotation
    public void turn(double p) {

        double backRightPower = p;

        if (p > 0)
            backRightPower += 0.2;
        else if (p < 0)
            backRightPower -= 0.2;

        front_right.setPower(p);
        front_left.setPower(-p); //flipped
        back_right.setPower(p);
        back_left.setPower(-p); //flipped

    }

    public void sleep(long ms) {
        runtime.reset();
        while (runtime.milliseconds() < ms && op.opModeIsActive() && !op.isStopRequested()) {
            op.idle();
        }
    }

    public double getAngle() {

        globalAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        return globalAngle;

    }
    public void resetAngle() {

        globalAngle = 0;
    }

    public void encoderDrive(double inches, double speed) {

        double ticks = constants.ticksPerInch * inches;

        double finalRightSpeed = speed;
        double finalBackRightSpeed = speed;
        double finalLeftSpeed = speed;

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(100);

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sleep(100);

        if (speed < 0) {
            finalRightSpeed -= 0.001;
            finalBackRightSpeed -= 0.08;
        }

        else {
            finalRightSpeed += 0.001;
            finalBackRightSpeed += 0.08;
        }


        front_left.setPower(finalLeftSpeed);
        front_right.setPower(finalRightSpeed);
        back_left.setPower(finalLeftSpeed);
        back_right.setPower(finalBackRightSpeed);

        while (op.opModeIsActive() && !op.isStopRequested() && Math.abs(front_left.getCurrentPosition())
                <= Math.abs(ticks) || Math.abs(front_right.getCurrentPosition()) <= Math.abs(ticks)) {

        }
        stopDriveChain();


    }


}
