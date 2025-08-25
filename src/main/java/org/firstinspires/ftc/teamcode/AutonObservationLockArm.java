package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="AutonObservationLockArm")
//@Disabled
@Config
public class AutonObservationLockArm extends LinearOpMode {
    private DcMotorEx flMotor, frMotor, blMotor, brMotor,armMotor;

    double forward = 0.5;
    double back = -0.5;
    double timeToMoveOneInch = 31.34;

    public static double f = 0.015;
    public static int target;

    public final double ticks_in_degrees = 700 / 180.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // create multiple telemetries and add to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Waiting");
        telemetry.update();

        // Initialization
        flMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frMotor = hardwareMap.get(DcMotorEx.class, "fr");
        blMotor = hardwareMap.get(DcMotorEx.class, "bl");
        brMotor = hardwareMap.get(DcMotorEx.class, "br");

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");

        waitForStart();

        target = armMotor.getCurrentPosition();

        Runnable lockArm = () -> {
            telemetry.addData("ArmCode", "Waiting");
            telemetry.update();

            long startTime = System.currentTimeMillis();
            long duration = 26000;

            while (System.currentTimeMillis() - startTime < duration) {
                lockArm();
            }
        };

        Thread lockArmThread = new Thread(lockArm);
        lockArmThread.start();

        //goto the block pushing position
        moveRobotForward(23);
        moveRobotLeft(68);
        moveRobotForward(27);

        //move three blocks
        moveRobotLeft(15);
        moveRobotBack(47);
        moveRobotForward(5);
        moveRobotRight(14);
        moveRobotBack(12);
        moveRobotLeft(20);
        moveRobotRight(5);
        moveRobotForward(50);
        moveRobotLeft(14);
        moveRobotBack(53);
        moveRobotForward(50);
        moveRobotLeft(10);
        moveRobotBack(53);

        //goto the parking in observation
        moveRobotForward(9);
        moveRobotRight( 141);
        moveRobotBack(13);
        stopRobot();

        telemetry.addData("Stop Robot", "Stopped");
        telemetry.update();
    }

    public void stopRobot() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }

    public void moveRobotRight(long inches) {
        double robotRunTimeInMilliSeconds = inches * timeToMoveOneInch;

        //set the motor power to 'power'
        flMotor.setPower(forward);
        frMotor.setPower(back);
        blMotor.setPower(back);
        brMotor.setPower(forward);
        sleep((long) robotRunTimeInMilliSeconds);
    }

    public void moveRobotLeft(long inches) {
        //robot at 2.75 speed has to run for 3 millisecond to cover a inch
        double robotRunTimeInMilliSeconds = inches * timeToMoveOneInch;

        //set the motor power to 'power'
        flMotor.setPower(back);
        frMotor.setPower(forward);
        blMotor.setPower(forward);
        brMotor.setPower(back);
        sleep((long) robotRunTimeInMilliSeconds);
    }

    public void moveRobotForward(long inches) {
        //robot at 2.75 speed has to run for 3 millisecond to cover a inch
        double robotRunTimeInMilliSeconds = inches * timeToMoveOneInch;

        //set the motor power to 'power'
        flMotor.setPower(forward);
        frMotor.setPower(forward);
        blMotor.setPower(forward);
        brMotor.setPower(forward);
        sleep((long) robotRunTimeInMilliSeconds);
    }

    public void moveRobotBack(long inches) {
        //robot at 2.75 speed has to run for 3 millisecond to cover a inch
        double robotRunTimeInMilliSeconds = inches * timeToMoveOneInch;
        ;
        //set the motor power to 'power'
        flMotor.setPower(back);
        frMotor.setPower(back);
        blMotor.setPower(back);
        brMotor.setPower(back);
        sleep((long) robotRunTimeInMilliSeconds);
    }

    public void lockArm() {
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        armMotor.setPower(ff);
    }
}
