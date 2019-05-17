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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Arm", group="Shmada Software")

public class Arm_Shake extends LinearOpMode {

    private DcMotor bicep = null;
    private DcMotor forearm = null;

    // Arm Speeds
    private double bicepSpeed = .15;
    private double forearmSpeed = .2;

    @Override
    public void runOpMode() {
        // Mech
        bicep = hardwareMap.get(DcMotor.class, "bicep");
        bicep.setDirection(DcMotor.Direction.FORWARD);

//        forearm = hardwareMap.get(DcMotor.class, "forearm");
//        forearm.setDirection(DcMotor.Direction.FORWARD);

        // Initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Active
        while (opModeIsActive()) {
            // LEFT JOY UP
            if(gamepad1.left_stick_y < 0){
                bicep.setPower(bicepSpeed);
            // LEFT JOY DOWN
            }else if(gamepad1.left_stick_y > 0){
                bicep.setPower(-bicepSpeed);
            }else{
                bicep.setPower(0);
                bicep.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // BUTTONS
            if(gamepad1.x){
                bicep.setPower(0);
                bicep.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // RIGHT JOY UP
//            if(gamepad1.right_stick_y < 0){
//                forearm.setPower(forearmSpeed);
//            // RIGHT JOY DOWN
//            }else if(gamepad1.right_stick_y > 0){
//                forearm.setPower(-forearmSpeed);
//            }else{
//                forearm.setPower(0);
//                forearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            }
        }
    }
}
