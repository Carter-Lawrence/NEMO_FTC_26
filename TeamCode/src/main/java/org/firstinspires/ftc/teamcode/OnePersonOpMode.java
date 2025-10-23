package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OnePersonOpMode", group="Linear OpMode")
public class OnePersonOpMode extends LinearOpMode {

    // DRIVE MOTORS
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // MECHANISMS
    private DcMotor intake;
    private DcMotorEx fly1, fly2;
    private Servo vertTrans;  // Linear actuator
    private Servo trans;      // Lift servo
    private Servo spin;       // Spin Dexter servo

    // BUTTON DEBOUNCE
    private boolean vertPressed = false;
    private boolean spinPressed = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // HARDWARE MAPPING
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        fly1       = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2       = hardwareMap.get(DcMotorEx.class, "fly2");
        intake     = hardwareMap.get(DcMotor.class, "in");

        vertTrans  = hardwareMap.get(Servo.class,"trans1");
        trans      = hardwareMap.get(Servo.class,"trans2");
        spin       = hardwareMap.get(Servo.class,"spin");

        // MOTOR DIRECTIONS
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // INITIAL SERVO POSITIONS
        vertTrans.setPosition(0);
        trans.setPosition(0);
        spin.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            // DRIVE CONTROL

            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x;

            double flPow = axial + lateral + yaw;
            double frPow = axial - lateral - yaw;
            double blPow = axial - lateral + yaw;
            double brPow = axial + lateral - yaw;

            // Normalize powers
            double max = Math.max(Math.abs(flPow), Math.abs(frPow));
            max = Math.max(max, Math.abs(blPow));
            max = Math.max(max, Math.abs(brPow));
            if (max > 1.0) {
                flPow /= max;
                frPow /= max;
                blPow /= max;
                brPow /= max;
            }

            frontLeft.setPower(flPow);
            frontRight.setPower(frPow);
            backLeft.setPower(blPow);
            backRight.setPower(brPow);


            // INTAKE / OUTTAKE

            // Speed adjust with R1/L1
            double intakeSpeed = 0.5 + 0.5 * (gamepad1.right_trigger - gamepad1.left_trigger); // 0 to 1

            if (gamepad1.x) {
                intake.setPower(intakeSpeed);  // Intake in
            } else if (gamepad1.y) {
                intake.setPower(-intakeSpeed); // Intake out
            } else {
                intake.setPower(0);
            }


            // FLYWHEELS

            if (gamepad1.right_bumper) {
                fly1.setPower(1.0);
                fly2.setPower(1.0);
            } else if (gamepad1.left_bumper) {
                fly1.setPower(0);
                fly2.setPower(0);
            }


            // VERTICAL SERVO (Linear actuator)

            if (gamepad1.a && !vertPressed) {
                vertTrans.setPosition(1.0);
                vertPressed = true;
            } else if (!gamepad1.a) {
                vertPressed = false;
            }


            // TRANS SERVO (lift)

            if (gamepad1.dpad_up) {
                trans.setPosition(1.0);
            } else if (gamepad1.dpad_down) {
                trans.setPosition(0.0);
            }


            // SPIN DEXTER SERVO

            if (gamepad1.b && !spinPressed) {
                double newPos = spin.getPosition() + 0.1;
                if (newPos > 1.0) newPos = 1.0;
                spin.setPosition(newPos);
                spinPressed = true;
            } else if (!gamepad1.b) {
                spinPressed = false;
            }
            // TELEMETRY
            telemetry.addData("Status", "Running");
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Fly Power", "Fly1: %.2f Fly2: %.2f", fly1.getPower(), fly2.getPower());
            telemetry.addData("VertTrans Pos", vertTrans.getPosition());
            telemetry.addData("Trans Pos", trans.getPosition());
            telemetry.addData("Spin Pos", spin.getPosition());
            telemetry.update();

            sleep(20);
        }
    }
}
