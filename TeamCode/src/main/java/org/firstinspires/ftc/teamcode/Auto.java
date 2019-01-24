package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoBots", group="Shmada Software")
public class Auto extends LinearOpMode {
    // Set Time
    private ElapsedTime runtime = new ElapsedTime();
    // Set Motors
    private DcMotor leftDrive = null;


    public void runOpMode() {

        leftDrive = hardwareMap.get(DcMotor.class, "left");



        leftDrive.setPower(1.0);
        while(opModeIsActive() && runtime.seconds() < 5){
            telemetry.update();
        }


    }

    public void move (double speed, double distance){

    }


}
