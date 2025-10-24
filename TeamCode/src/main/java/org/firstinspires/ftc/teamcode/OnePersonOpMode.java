package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private DcMotor fly1, fly2;
    private Servo vertTrans;  // Vertical actuator
    private CRServo trans;    // Continuous lift servo
    private Servo spin;       // Spin Dexter servo

    // BUTTON DEBOUNCE
    private boolean vertPressed = false;
    private boolean spinPressed = false;
    private boolean flyOn = false;

    private double spinZero = 20.0 / 180.0; // start at 20°

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // HARDWARE MAPPING
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        fly1       = hardwareMap.get(DcMotor.class, "fly1");
        fly2       = hardwareMap.get(DcMotor.class, "fly2");
        intake     = hardwareMap.get(DcMotor.class, "in");

        vertTrans  = hardwareMap.get(Servo.class,"trans1");
        trans      = hardwareMap.get(CRServo.class,"trans2");
        spin       = hardwareMap.get(Servo.class,"spin");

        // DIRECTIONS
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        vertTrans.setPosition(0);
        trans.setPower(0);
        spin.setPosition(spinZero);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ---------- DRIVE (optional) ----------
            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x;

            double flPow = axial + lateral + yaw;
            double frPow = axial - lateral - yaw;
            double blPow = axial - lateral + yaw;
            double brPow = axial + lateral - yaw;

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

            // ---------- INTAKE ----------
            double intakeSpeed = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -1.0, 1.0);
            intake.setPower(intakeSpeed);

            // ---------- FLYWHEELS ----------
            if(gamepad1.right_bumper) flyOn = true;
            if(gamepad1.left_bumper) flyOn = false;

            if(flyOn){
                fly1.setPower(1.0);
                fly2.setPower(1.0);
            } else {
                fly1.setPower(0);
                fly2.setPower(0);
            }

            // ---------- VERTICAL SERVO ----------
            if(gamepad1.a && !vertPressed){
                vertTrans.setPosition(0.0); // move down
                vertPressed = true;
            } else if(!gamepad1.a){
                vertPressed = false;
            }

            // ---------- TRANS SERVO ----------
            if(gamepad1.dpad_up){
                trans.setPower(1.0);
            } else {
                trans.setPower(0);
            }

            // ---------- SPIN DEXTER ----------
            if(gamepad1.b && !spinPressed){
                spinZero = 0; // reset relative zero
                double newPos = spin.getPosition() + 120.0/180.0;
                if(newPos > 1.0) newPos -= 1.0; // wrap
                spin.setPosition(newPos);
                spinPressed = true;
            } else if(!gamepad1.b){
                spinPressed = false;
            }

            // ---------- TELEMETRY ----------
            telemetry.addData("Intake", intake.getPower());
            telemetry.addData("Flywheels", flyOn);
            telemetry.addData("VertTrans", vertTrans.getPosition());
            telemetry.addData("Spin", spin.getPosition());
            telemetry.update();

            sleep(20);
        }
    }
}
