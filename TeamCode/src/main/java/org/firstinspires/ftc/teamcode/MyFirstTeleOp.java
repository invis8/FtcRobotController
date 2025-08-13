package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class MyFirstTeleOp extends LinearOpMode {

    // Timer, motors, servos, and button-state trackers
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armUp = null;
    private DcMotor armDown = null;
    private DcMotor armUp2 = null;
    private DcMotor armDown2 = null;
    private Servo servo1 = null;
    private Servo servo2 = null;

    private boolean lastXButtonState = false;
    private boolean lastBButtonState = false;
    private boolean lastAButtonState = false;
    private boolean lastYButtonState = false;

    private double servo1Position = 0.5;
    private double servo2Position = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Map motors and servos to configuration names
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armUp      = hardwareMap.get(DcMotor.class, "arm");
        armDown    = hardwareMap.get(DcMotor.class, "arm");
        armUp2     = hardwareMap.get(DcMotor.class, "arm2");
        armDown2   = hardwareMap.get(DcMotor.class, "arm2");
        servo1     = hardwareMap.get(Servo.class, "servo1");
        servo2     = hardwareMap.get(Servo.class, "servo2");

        // Reverse wheel directions so forward sticks make robot go forward
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // Main control loop
        while (opModeIsActive()) {
            // Tank drive: left stick controls left wheel, right stick controls right wheel
            double leftPower  = gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Raise arm when left bumper held
            armUp.setDirection(DcMotor.Direction.FORWARD);
            armUp2.setDirection(DcMotor.Direction.REVERSE);
            if (gamepad1.left_bumper) {
                armUp.setPower(0.5);
                armUp2.setPower(0.5);
            } else {
                armUp.setPower(0);
                armUp2.setPower(0);
            }

            // Lower arm when right bumper held, otherwise hold position slowly
            armDown.setDirection(DcMotor.Direction.REVERSE);
            armDown2.setDirection(DcMotor.Direction.FORWARD);
            if (gamepad1.right_bumper) {
                armDown.setPower(0.4);
                armDown2.setPower(0.4);
            } else {
                armDown.setPower(0.17);
                armDown2.setPower(0.17);
            }

            // Servo1 toggle between two positions on X button press
            servo1.setPosition(servo1Position);
            if (gamepad1.x && !lastXButtonState) {
                servo1Position = (servo1Position == 0.5) ? 0.4 : 0.5;
                servo1.setPosition(servo1Position);
                lastXButtonState = true;
            } else if (!gamepad1.x) {
                lastXButtonState = false;
            }

            // Reset servo1 to default on B press
            if (gamepad1.b && !lastBButtonState) {
                servo1.setPosition(0.5);
                lastBButtonState = true;
            } else if (!gamepad1.b) {
                lastBButtonState = false;
            }

            // Fine-adjust servo2 up with A, down with Y
            if (gamepad1.a && !lastAButtonState) {
                servo2Position = Range.clip(servo2Position + 0.0008, 0, 1);
                servo2.setPosition(servo2Position);
                lastAButtonState = true;
            } else if (!gamepad1.a) {
                lastAButtonState = false;
            }

            if (gamepad1.y && !lastYButtonState) {
                servo2Position = Range.clip(servo2Position - 0.0008, 0, 1);
                servo2.setPosition(servo2Position);
                lastYButtonState = true;
            } else if (!gamepad1.y) {
                lastYButtonState = false;
            }

            // Display runtime and motor powers
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Wheel Power", "L(%.2f) R(%.2f)", leftPower, rightPower);
            telemetry.addData("Arm Up Power", armUp.getPower());
            telemetry.addData("Arm Down Power", armDown.getPower());
            telemetry.update();
        }
    }
}