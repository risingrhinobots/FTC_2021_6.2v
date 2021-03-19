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

package org.firstinspires.ftc.teamcode.telop.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop_Team_16310_Linear", group="Iterative Opmode")
//@Disabled
public class Teleop_Team_16310_Linear extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx frontleftDrive = null;
    private DcMotorEx frontrightDrive = null;
    private DcMotorEx backleftDrive = null;
    private DcMotorEx backrightDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontleftDrive  = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontrightDrive = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backleftDrive  = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backrightDrive = hardwareMap.get(DcMotorEx.class, "BackRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontleftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontrightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backleftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backrightDrive.setDirection(DcMotorEx.Direction.REVERSE);

        frontleftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        frontleftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontrightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backleftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backrightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        // Tell the driver that initialization is complete.

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontleftPower;
        double frontrightPower;
        double backleftPower;
        double backrightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //First we had three variables:
        //drive = - leftjoystick y
        //turn = rightjoystick x1
        //strafe = leftjoystick x2
        //motor[frontRight] = -Y1 - X2 - X1;
        //motor[backRight] =  -Y1 + X2 + X1;
        //motor[frontLeft] = -Y1 - X2 + X1;
        //motor[backLeft] =  -Y1 + X2 - X1;


        double drive = 0.8*(gamepad1.left_stick_y);
        double turn  =  0.6 * (-gamepad1.right_stick_x);
        double strafe = 0.8*(-gamepad1.left_stick_x);


        frontleftPower    = Range.clip(-drive - turn - strafe, -0.6, 0.6) ;
        frontrightPower   = Range.clip(-drive + turn + strafe, -0.6, 0.6) ;
        backleftPower    = Range.clip(-drive - turn + strafe, -0.6, 0.6) ;
        backrightPower   = Range.clip(-drive + turn - strafe, -0.6, 0.6) ;
        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        frontleftDrive.setPower(frontleftPower);
        frontrightDrive.setPower(frontrightPower);
        backleftDrive.setPower(backleftPower);
        backrightDrive.setPower(backrightPower);

        if (gamepad1.x){
            frontleftDrive.setPower(-1);
            frontrightDrive.setPower(1);
            backleftDrive.setPower(1);
            backrightDrive.setPower(-1);
        }

        if(gamepad1.b){
            frontleftDrive.setPower(1);
            frontrightDrive.setPower(-1);
            backleftDrive.setPower(-1);
            backrightDrive.setPower(1);
        }

        telemetry.addData("Motors", "Frontleft (%.2f), Frontright (%.2f), Backleft (%.2f), Backright (%.2f)", frontleftPower, frontrightPower, backleftPower,backrightPower);
        telemetry.addData("velocity", frontleftDrive.getVelocity());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */

}

