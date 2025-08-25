package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Mechanum Drive Code PID")
public class MechanumDriveCodePID extends LinearOpMode {
    private DcMotorEx flMotor, frMotor, blMotor, brMotor;
    private DcMotorEx slideMotor, armMotor, linearAccelerator;
    private CRServo sampleIntakeServo;
    private Servo leftSpecimenIntakeServo, rightSpecimenIntakeServo;

    public static double p = 0.003, i = 0.5, d = 1.5;
    public static double f = 0.001;
    public static double target = 0.0;

    // Slide and rotation of slide code
    public void arm() {
        // PID control
        PIDController controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        int armPosition = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPosition, target);
        double ticks_in_degrees = 700 / 180.0;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double armPower = pid + ff;

        target = target + gamepad2.right_stick_y * 5;
        armMotor.setPower(armPower);

        telemetry.addData("armPosition", armPosition);
        telemetry.addData("target", target);
        telemetry.addData("pid", armPower);
        telemetry.addData("gamepad value", gamepad2.right_stick_y);

    }

    public void armWithoutPID() {
        double ticks_in_degrees = 700 / 180.0;
        double currentPosition = armMotor.getCurrentPosition();
        double ff = Math.cos(Math.toRadians(currentPosition / ticks_in_degrees)) * f;

        double controllerPower = gamepad2.right_stick_y;

        if(controllerPower != 0) {
            armMotor.setPower(controllerPower + ff);
        } else {
            armMotor.setPower(ff);
        }

        telemetry.addData("controller power", controllerPower);
        telemetry.addData("feedforward", ff);
        telemetry.addData("arm Position", currentPosition);

    }

    public void slide() {
        double slidePower = 0.6 * -gamepad2.left_stick_y;
        slideMotor.setPower(slidePower);
    }

    // Intakes separated from arm because of future PID implementation on arm
    public void sampleIntake() {
        double sampleIntakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        sampleIntakeServo.setPower(sampleIntakePower);
    }

    public void specimenIntake() {
        if(gamepad1.a) {
            rightSpecimenIntakeServo.setPosition(1);
            leftSpecimenIntakeServo.setPosition(0);
        }

        if(gamepad1.b) {
            rightSpecimenIntakeServo.setPosition(0);
            leftSpecimenIntakeServo.setPosition(1);

        }
        double leftSpecimenIntakeServoPosition = leftSpecimenIntakeServo.getPosition();
        double rightSpecimenIntakeServoPosition = rightSpecimenIntakeServo.getPosition();
        telemetry.addData("leftSpecimenIntakeServoPosition", leftSpecimenIntakeServoPosition);
        telemetry.addData("rightSpecimenIntakeServoPosition", rightSpecimenIntakeServoPosition);
    }

    public void linearAccelerator() {
        double acceleratorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        linearAccelerator.setPower(acceleratorPower);
    }


    // Drive code
    public void drive() {
        //get joystick values
        double xPower = 0.7 * gamepad1.left_stick_x;
        double yPower = 0.7 * -gamepad1.left_stick_y;
        double yaw = 0.7 * gamepad1.right_stick_x;

        //calculate powers
        double flPower = xPower + yPower + yaw;
        double frPower = -xPower + yPower - yaw;
        double blPower = -xPower + yPower + yaw;
        double brPower = xPower + yPower - yaw;

        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));
        if(maxPower > 1.0) {
            flPower /= maxPower;
            blPower /= maxPower;
            frPower /= maxPower;
            brPower /= maxPower;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    public void initialization() {
        // Initialization
        flMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frMotor = hardwareMap.get(DcMotorEx.class, "fr");
        blMotor = hardwareMap.get(DcMotorEx.class, "bl");
        brMotor = hardwareMap.get(DcMotorEx.class, "br");

        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reverse direction of motors
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSpecimenIntakeServo = hardwareMap.get(Servo.class, "intakeServoLeft");
        rightSpecimenIntakeServo = hardwareMap.get(Servo.class, "intakeServoRight");

        sampleIntakeServo = hardwareMap.get(CRServo.class, "intakeServoClaw");

        linearAccelerator = hardwareMap.get(DcMotorEx.class, "acc");
    }

    public void runOpMode() {
        // init
        initialization();

        // create multiple telemetries and add to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Waiting");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            this.drive();
            this.arm();
            this.slide();
            this.specimenIntake();
            this.sampleIntake();
            this.linearAccelerator();
            telemetry.update();
        }
    }
}