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

package org.firstinspires.ftc.teamcode.auto.mecanum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwarePushbot_TC;
import org.firstinspires.ftc.teamcode.Mecanum_hardware;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Config
@Autonomous(name="PID_mecanum", group="Pushbot")
//@Disabled
public class PID_mecanum extends LinearOpMode {

    /* Declare OpMode members. */
    Mecanum_hardware mecanum   = new Mecanum_hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    FtcDashboard dashboard;

    public static double kP = 0.05;
    public static double kI = 0.1;
    public static double kD = 0.00;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;   // 1  // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1500;
    static final double     TURN_SPEED              = 0.3;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        mecanum.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        mecanum.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mecanum.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanum.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanum.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanum.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          mecanum.frontRight.getCurrentPosition(),
                          mecanum.frontRight.getCurrentPosition());
        telemetry.update();

        dashboard = FtcDashboard.getInstance();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        encoderPID(65,65,65,65);
        sleep(2000);
        //encoderPID(-11,11);

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderPID(double frontleftInches, double frontrightInches, double backleftInches, double backrightInches) {

            runtime.reset();

            double errorsumfrontleft = 0;
            double errorsumfrontright = 0;
            double errorsumbackleft = 0;
            double errorsumbackright = 0;

            double lasttime = 0;

            double lasterrorfrontleft = 0;
            double lasterrorfrontright = 0;
            double lasterrorbackleft = 0;
            double lasterrorbackright = 0;

            double iLimit = 10;

            runtime.startTime();

            while (opModeIsActive()){

                double errorfrontleft = frontleftInches - (mecanum.frontLeft.getCurrentPosition()/COUNTS_PER_INCH);
                double errorfrontright = frontrightInches - (mecanum.frontRight.getCurrentPosition()/COUNTS_PER_INCH);
                double errorbackleft = backleftInches - (mecanum.backLeft.getCurrentPosition()/COUNTS_PER_INCH);
                double errorbackright = backrightInches - (mecanum.backRight.getCurrentPosition()/COUNTS_PER_INCH);
                double dt = runtime.seconds() - lasttime;

                if(Math.abs(errorfrontleft) < iLimit){
                    errorsumfrontleft += errorfrontleft*dt;
                }

                if(Math.abs(errorfrontright) < iLimit){
                    errorsumfrontright += errorfrontright*dt;
                }

                if(Math.abs(errorbackleft)< iLimit){
                    errorsumbackleft += errorbackleft*dt;
                }

                if (Math.abs(errorbackright)<iLimit){
                    errorsumbackright += errorbackright*dt;
                }

                double errorateleft = (errorfrontleft - lasterrorfrontleft) / dt;
                double errorrateright = (errorfrontright - lasterrorfrontright) / dt;
                double errorratebackleft = (errorbackleft - lasterrorbackleft) / dt;
                double errorratebackright = (errorbackright - lasterrorbackright) / dt;

                double frontleftspeed = kP*errorfrontleft + kI*errorsumfrontleft + kD*errorateleft;
                double frontrightspeed = kP*errorfrontright + kI*errorsumfrontright + kD*errorrateright;
                double backleftspeed = kP*errorbackleft + kI*errorsumbackleft + kD*errorratebackleft;
                double backrightspeed = kP*errorbackright + kI*errorsumbackright + kD*errorratebackright;


                double finalfrontleftspeed = Range.clip(frontleftspeed,-1500,1500);
                double finalfrontrightspeed = Range.clip(frontrightspeed,-1500,1500);
                double finalbackleftspeed = Range.clip(backleftspeed,-1500,1500);
                double finalbackrightspeed = Range.clip(backrightspeed,-1500,1500);

                mecanum.frontRight.setVelocity(finalfrontrightspeed);
                mecanum.frontLeft.setVelocity(finalbackleftspeed);
                mecanum.backRight.setVelocity(finalbackrightspeed);
                mecanum.backLeft.setVelocity(finalbackleftspeed);

                lasttime = runtime.seconds();
                lasterrorfrontleft = errorfrontleft;
                lasterrorfrontright = errorfrontright;
                lasterrorbackleft = errorbackleft;
                lasterrorbackright = errorbackright;

                //telemetry.addData("right target",newRightTarget);
                //telemetry.addData("left target", newLeftTarget);

                telemetry.addData("leftspeed",finalfrontleftspeed);
                telemetry.addData("rightspeed", finalfrontrightspeed);




            }
    }
}
