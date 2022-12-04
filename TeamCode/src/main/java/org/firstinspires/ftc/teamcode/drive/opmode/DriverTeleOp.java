package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TELEOP")
public class DriverTeleOp extends Hardware {
    double leftServoPos = 0;
    double rightServoPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        initialize();
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
                turnPower = -.3;
            } else if (gamepad2.dpad_right){
                turnPower = .4;
            } else {
                turnPower = 0;
            }
            if (gamepad2.a){
                leftServoPos = 0.2;
                rightServoPos = 0.9;
            } else if (gamepad2.b){
                leftServoPos = 0.3;
                rightServoPos = 0.8;
            }
            setServos(leftServoPos,rightServoPos);
            turnTurret(turnPower);
            moveSliders(sliderPower);
            drive(y,x, rx);
        }
    }
}