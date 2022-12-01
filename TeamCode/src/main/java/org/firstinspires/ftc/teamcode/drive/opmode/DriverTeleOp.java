package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TELEOP")
public class DriverTeleOp extends Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            int sliderPower;
            double turnPower;

            if (gamepad2.dpad_up){
                sliderPower = 1;
            } else if (gamepad2.dpad_down){
                sliderPower = -1;
            } else {
                sliderPower = 0;
            }

            if (gamepad2.dpad_left){
                turnPower = -.5;
            } else if (gamepad2.dpad_right){
                turnPower = .5;
            } else {
                turnPower = 0;
            }

            turnTurret(turnPower);
            moveSliders(sliderPower);
            drive(y,x, rx);
        }
    }
}