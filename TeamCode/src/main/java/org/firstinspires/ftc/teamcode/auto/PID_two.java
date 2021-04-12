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

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwarePushbot_TC;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the 
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
@Autonomous(name="PID_two", group="Pushbot")
//@Disabled
public class PID_two extends LinearOpMode {

    public DcMotorEx  frontLeft   = null;
    public DcMotorEx  frontRight  = null;
    public DcMotorEx  backLeft  = null;
    public DcMotorEx  backRight  = null;
    
    private ElapsedTime     runtime = new ElapsedTime();

    FtcDashboard dashboard;

    public static double kP = 0.05;
    public static double kI = 0.1;
    public static double kD = 0.05;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;   // 1  // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1200;
    static final double     TURN_SPEED              = 1000;
    final double tolerance= 0.00000000001;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        frontLeft  = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft =  hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          frontRight.getCurrentPosition(),
                          frontRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        encoderPID(40,40);
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
    public void encoderPID(double frontleftInches, double frontrightInches) {

        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            runtime.reset();

            double errorsumleft = 0;
            double errorsumright = 0;

            double lasttime = 0;

            double lasterrorleft = 0;
            double lasterrorright = 0;

            double iLimit = 10;

            runtime.startTime();

            while (opModeIsActive()){

                double errorleft = frontleftInches - (frontLeft.getCurrentPosition()/COUNTS_PER_INCH);
                double errorright = frontrightInches - (frontRight.getCurrentPosition()/COUNTS_PER_INCH);
                double dt = runtime.seconds() - lasttime;

                if(Math.abs(errorleft) < iLimit){
                    errorsumleft += errorleft*dt;
                }

                if(Math.abs(errorright) < iLimit){
                    errorsumright += errorright*dt;
                }

                double errorateleft = (errorleft - lasterrorleft) / dt;
                double errorrateright = (errorright - lasterrorright) / dt;

                double leftspeed = (kP*errorleft + kI*errorsumleft + kD*errorateleft);
                double rightspeed = (kP*errorright + kI*errorsumright + kD*errorrateright);

                double finalleftspeed = Range.clip(leftspeed,-0.8,0.8);
                double finalrightspeed = Range.clip(rightspeed,-0.8,0.8);

                frontRight.setPower(finalrightspeed);
                frontLeft.setPower(finalleftspeed);
                backRight.setPower(finalrightspeed);
                backLeft.setPower(finalleftspeed);

                lasttime = runtime.seconds();
                lasterrorleft = errorleft;
                lasterrorright = errorright;

                //telemetry.addData("right target",newRightTarget);
                //telemetry.addData("left target", newLeftTarget);

                telemetry.addData("leftspeed",finalleftspeed);
                telemetry.addData("rightspeed", finalrightspeed);


            }
    }
}
