/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions in the documentation and/or other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED BY ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OnePersonOpMode", group="Linear OpMode")
public class OnePersonOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive motors
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // Flywheels
    private DcMotor fly1 = null;
    private DcMotor fly2 = null;

    // Intake
    private DcMotor intake = null;

    // Servos
    private Servo vertTrans = null;
    private Servo trans = null;
    private Servo spin = null;

    // CONTROL VARS
    boolean lb1Pressed = false;
    boolean rb1Pressed = false;
    boolean b1Pressed = false;
    boolean a1Pressed = false;
    boolean x1Pressed = false;
    boolean y1Pressed = false;
    boolean down1Pressed = false;
    boolean up1Pressed = false;
    boolean right1Pressed = false;
    boolean left1Pressed = false;
    boolean is = false

    boolean l1

    boolean lb2Pressed = false;
    boolean rb2Pressed = false;
    boolean b2Pressed = false;
    boolean a2Pressed = false;
    boolean x2Pressed = false;
    boolean y2Pressed = false;
    boolean down2Pressed = false;
    boolean up2Pressed = false;
    boolean right2Pressed = false;
    boolean left2Pressed = false;



    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        fly1 = hardwareMap.get(DcMotor.class, "fly1");
        fly2 = hardwareMap.get(DcMotor.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "in");
        vertTrans = hardwareMap.get(Servo.class, "trans1");
        trans = hardwareMap.get(Servo.class, "trans2");
        spin = hardwareMap.get(Servo.class, "spin");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double intakeSpeed = 0.5;
        double flySpeed = 0.8;

        while (opModeIsActive()) {
            // Drive controls
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Intake toggle
            if (gamepad1.x && !x1Pressed) {
                intake.setPower(intakeSpeed);
            }
            if (gamepad1.y && !y1Pressed) {
                intake.setPower(-intakeSpeed);
            }

            // Intake speed adjust
            if (gamepad1.r1) {
                intakeSpeed = Range.clip(intakeSpeed + 0.01, 0, 1);
            }
            if (gamepad1.l1) {
                intakeSpeed = Range.clip(intakeSpeed - 0.01, 0, 1);
            }

            // Flywheel toggle
            if (gamepad1.r2) {
                fly1.setPower(flySpeed);
                fly2.setPower(flySpeed);
            } else if (gamepad1.l2) {
                fly1.setPower(-flySpeed);
                fly2.setPower(-flySpeed);
            } else {
                fly1.setPower(0);
                fly2.setPower(0);
            }

            // Flywheel speed adjust
            if (gamepad1.right_bumper) {
                flySpeed = Range.clip(flySpeed + 0.01, 0, 1);
            }
            if (gamepad1.left_bumper) {
                flySpeed = Range.clip(flySpeed - 0.01, 0, 1);
            }

            // VertTrans Servo
            if (gamepad1.a && !a1Pressed) {
                vertTrans.setPosition(1);
            } else {
                vertTrans.setPosition(0);
            }

            // Trans Servo (up/down)
            if (gamepad1.dpad_up) {
                trans.setPosition(1);
            } else if (gamepad1.dpad_down) {
                trans.setPosition(0);
            }

            // Spin Dexter
            if (gamepad1.b && !b1Pressed) {
                spin.setPosition(spin.getPosition() + 0.1);
            }

            // update previous button states (only for gamepad1)
            lb1Pressed = gamepad1.left_bumper;
            rb1Pressed = gamepad1.right_bumper;
            b1Pressed = gamepad1.b;
            a1Pressed = gamepad1.a;
            x1Pressed = gamepad1.x;
            y1Pressed = gamepad1.y;
            down1Pressed = gamepad1.dpad_down;
            up1Pressed = gamepad1.dpad_up;
            left1Pressed = gamepad1.dpad_left;
            right1Pressed = gamepad1.dpad_right;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Intake Speed", intakeSpeed);
            telemetry.addData("Fly Power", fly1.getPower());
            telemetry.addData("Fly Speed", flySpeed);
            telemetry.addData("VertTrans", vertTrans.getPosition());
            telemetry.addData("Trans", trans.getPosition());
            telemetry.addData("Spin", spin.getPosition());
            telemetry.update();
        }
    }
}
