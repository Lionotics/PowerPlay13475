package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

@TeleOp(name = "TELEOP")
public class DriverTeleOp extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotObject robot = new RobotObject(this);

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        robot.init();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            if (gamepad2.dpad_up){
                robot.sliderPower = 1;
            } else if (gamepad2.dpad_down){
                robot.sliderPower = -1;
            } else {
                robot.sliderPower = 0;
            }

            if (gamepad2.dpad_left){
                robot.turretPower = -.3;
            } else if (gamepad2.dpad_right){
                robot.turretPower = .4;
            } else {
                robot.turretPower = 0;
            }

            if (gamepad2.a){
                // open
                robot.servoPos = 0.1;
                robot.updateServos();
            } else if (gamepad2.b){
                // close
                robot.servoPos = 0.2;
                robot.updateServos();
            }

            telemetry.addLine(String.valueOf(robot.servoPos));
            telemetry.update();
            robot.updateTurret();
            robot.updateSliders();

            robot.drive(y,x, rx);
        }
    }
}