/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotObject {

    // Define Drive constants They are public, so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    // Define Motor and Servo objects  (Make them private, so they can't be accessed externally)
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, liftOne, liftTwo, turret;
    private Servo leftServo, rightServo;
    private BNO055IMU imu;
    double servoPos;
    double sliderPower;
    double turretPower;
    //Define positions on the field for rr
    Pose2d startingPos;
    Vector2d parkPos;
    //TODO: Actually set these
    final Pose2d LEFT_STARTING = new Pose2d(35,35,toRadians(0));
    final Pose2d RIGHT_STARTING = new Pose2d(35,35,toRadians(0));
    final Vector2d LEFT_PARK = new Vector2d(0,0);
    final Vector2d RIGHT_PARK = new Vector2d(0,0);
    final Vector2d MIDDLE_PARK = new Vector2d(0,0);

    static final double     COUNTS_PER_INCH         = (537.7 * 1) /
            (3.779528 * Math.PI);

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotObject(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * All the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        frontLeft = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("frontLeft");
        frontRight = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("frontRight");
        backLeft = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("backLeft");
        backRight = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("backRight");
        liftOne = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("liftOne");
        liftTwo = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("liftTwo");
        turret = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("turret");
        leftServo = myOpMode.hardwareMap.servo.get("leftServo");
        rightServo = myOpMode.hardwareMap.servo.get("rightServo");

        // Reverse all right things
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
//        leftServo.setPosition(MID_SERVO);
//        rightServo.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates all four motor powers required to achieve the requested robot motions
     * Then sends these power levels to the motors.
     */
    public void drive(double y, double x, double rx){
        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
    public void sketchyEncoderDrive( double inches, double frontRightPower, double frontLeftPower, double backLeftPower, double backRightPower){
        // stop and reset the encoders? Maybe not. Might want to get position and add from there
            double newFRTarget = 0;
              if(frontRightPower > 0) {
                  newFRTarget = frontRight.getCurrentPosition() + (inches * COUNTS_PER_INCH);
              } else if(frontRightPower < 0){
                  newFRTarget = frontRight.getCurrentPosition() - (inches * COUNTS_PER_INCH);
              }

            // Run with encoders
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myOpMode.telemetry.addData("Current", frontRight.getCurrentPosition());
            myOpMode.telemetry.addData("Target",newFRTarget);
            myOpMode.telemetry.update();
            myOpMode.sleep(1000);


            // Set powers. For now, I'm setting to maxPower, so be careful. This is called sketchy for a reason
            frontRight.setPower(frontRightPower);
            frontLeft.setPower(-frontLeftPower);
            backRight.setPower(backRightPower);
            backLeft.setPower(-backLeftPower);


            if(frontRightPower > 0) {
                while (newFRTarget - Math.abs(frontRight.getCurrentPosition()) > 15) {
                    myOpMode.telemetry.addData("Current", frontRight.getCurrentPosition());
                    myOpMode.telemetry.addData("Target", newFRTarget);
                    myOpMode.telemetry.update();

                }
            } else if(frontRightPower < 0){
                while ( frontRight.getCurrentPosition() - newFRTarget > 15) {
                    myOpMode.telemetry.addData("Current", frontRight.getCurrentPosition());
                    myOpMode.telemetry.addData("Target", newFRTarget);
                    myOpMode.telemetry.update();

                }
            }
            // Set Zero Power
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);


            // Go back to no encoders
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    public void extraSketchyEncoderDrive( double inches, double frontRightPower, double frontLeftPower, double backLeftPower, double backRightPower){
        // stop and reset the encoders? Maybe not. Might want to get position and add from there
        double newFLTarget;
        newFLTarget = frontLeft.getCurrentPosition()     +  (inches * COUNTS_PER_INCH);

        // Run with encoders
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myOpMode.telemetry.addData("Current", frontLeft.getCurrentPosition());
        myOpMode.telemetry.addData("Target",newFLTarget);
        myOpMode.telemetry.update();


        // Set powers. For now, I'm setting to maxPower, so be careful. This is called sketchy for a reason
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(-frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(-backLeftPower);



        while (newFLTarget - Math.abs(frontLeft.getCurrentPosition()) > 15) {
            myOpMode.telemetry.addData("Current", frontLeft.getCurrentPosition());
            myOpMode.telemetry.addData("Target",newFLTarget);
            myOpMode.telemetry.update();

        }
        // Set Zero Power
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);


        // Go back to no encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
        public void encoderDriveAnd(double maxPower, double frontRightInches, double frontLeftInches, double backLeftInches, double backRightInches){
            // stop and reset the encoders? Maybe not. Might want to get position and add from there
            double newFRTarget;
            double newFLTarget;
            double newBLTarget;
            double newBRTarget;

            //calculate and set target positions

            newFRTarget = frontRight.getCurrentPosition()     +  (frontRightInches * COUNTS_PER_INCH);
            newFLTarget = frontLeft.getCurrentPosition()     +  (frontLeftInches * COUNTS_PER_INCH);
            newBLTarget = backLeft.getCurrentPosition()     +  (backLeftInches * COUNTS_PER_INCH);
            newBRTarget = backRight.getCurrentPosition()     + (backRightInches * COUNTS_PER_INCH);

            backRight.setTargetPosition((int)(newBRTarget));
            frontRight.setTargetPosition((int)(newFRTarget));
            frontLeft.setTargetPosition((int)(newFLTarget));
            backLeft.setTargetPosition((int)(newBLTarget));

            // Run to position
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set powers. For now, I'm setting to maxPower, so be careful.
            // In the future I'd like to add some acceleration control through powers, which
            // should help with encoder accuracy. Stay tuned.
            frontRight.setPower(maxPower);
            frontLeft.setPower(maxPower);
            backRight.setPower(maxPower);
            backLeft.setPower(maxPower);

            while ((frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy() )) {

            }
            // Set Zero Power
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);

            // Go back to Run_Using_Encoder
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    /**
     * Moves the slider the inputted amount.
     * The multiplier is in case the sliders need to be slowed down or sped up
     * */
    public void updateSliders(){
        liftOne.setPower(sliderPower);
        liftTwo.setPower(sliderPower);
    }
    /**
     * Sets the servos to the inputted position.
     * It doesn't have to reverse the right, because that is done in init
     * */
    public void updateServos(){
        leftServo.setPosition(servoPos);
        rightServo.setPosition(servoPos);
    }
    /**
     * Sets the power of the turret
     * */
    public void updateTurret (){
        turret.setPower(turretPower);
    }
    // TODO: Add a drive method which considers goofy tilted robot
}