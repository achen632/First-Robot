/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive Code", group="Shmada Software")

public class First_Code extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor latchMech = null;

    // Drive Speeds
    private double dpadSpeed = .3;
    private double stickSpeed = .5;

    // Turn Speeds
    private double turnMultiplier = .8;
    private double dpadTurnSpeed = dpadSpeed * turnMultiplier;
    private double stickTurnSpeed = stickSpeed * turnMultiplier;

    // Mech Speeds
    private double maxSpeed = 1;

    @Override
    public void runOpMode() {

        // Initial

        // Drive
        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        rightDrive = hardwareMap.get(DcMotor.class, "right");
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Mech
        latchMech = hardwareMap.get(DcMotor.class, "latch");
        latchMech.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Active
        while (opModeIsActive()) {


            if(gamepad1.right_trigger > 0){
                telemetry.addData("Speed", "Sonic");
                telemetry.update();
                stickSpeed = 1;
                dpadSpeed = 0.6;
            }else{
                stickSpeed = 0.5;
                dpadSpeed = 0.3;
                telemetry.addData("Speed", "Snail");
                telemetry.update();
            }

            // DUAL STICK DRIVE

            if(gamepad1.left_stick_y < 0){
                rightDrive.setPower(stickSpeed);
                leftDrive.setPower(stickSpeed);

            }else if(gamepad1.left_stick_y > 0){
                rightDrive.setPower(-stickSpeed);
                leftDrive.setPower(-stickSpeed);

            }else if(gamepad1.right_stick_x > 0){
                rightDrive.setPower(-stickTurnSpeed);
                leftDrive.setPower(stickTurnSpeed);

            }else if(gamepad1.right_stick_x < 0) {
                rightDrive.setPower(stickTurnSpeed);
                leftDrive.setPower(-stickTurnSpeed);

            // DPAD DRIVE
            }else if(gamepad1.dpad_up){
                rightDrive.setPower(dpadSpeed);
                leftDrive.setPower(dpadSpeed);

            }else if(gamepad1.dpad_down){
                rightDrive.setPower(-dpadSpeed);
                leftDrive.setPower(-dpadSpeed);

            }else if(gamepad1.dpad_left){
                rightDrive.setPower(dpadTurnSpeed);
                leftDrive.setPower(-dpadTurnSpeed);

            }else if(gamepad1.dpad_right){
                rightDrive.setPower(-dpadTurnSpeed);
                leftDrive.setPower(dpadTurnSpeed);
            }else{
                rightDrive.setPower(0);
                leftDrive.setPower(0);
            }

            ////////////// MECH CODE //////////////

            // LATCH MECH

            if(gamepad1.right_bumper){
                latchMech.setPower(maxSpeed);
            }else if(gamepad1.left_bumper){
                latchMech.setPower(-maxSpeed);
            }else{
                latchMech.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }
}
