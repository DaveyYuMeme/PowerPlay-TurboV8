package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TurboTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        Servo Dep = hardwareMap.servo.get("Deposit");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.a){
                Dep.setPosition(0.5);
            }
            if(gamepad1.b){
                Dep.setPosition(0);
            }

            //telemetry.addData("fuck you",1);
            telemetry.addLine("* Reading this doesn't seem like the best use of your time.");
            telemetry.update();
        }
    }
}