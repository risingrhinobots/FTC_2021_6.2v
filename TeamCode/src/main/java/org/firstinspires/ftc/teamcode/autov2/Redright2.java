/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autov2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwarePushbot_TC;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Red right 2", group = "UltimateGame")

public class Redright2 extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    /* Declare OpMode members. */
    HardwarePushbot_TC robot   = new HardwarePushbot_TC();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;   // 1  // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1600;
    static final double     TURN_SPEED              = 1300;

    double  armPosition, gripPosition, gatePosition,guidePosition;
    double armMinPosition, armMaxPosition;


    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        armMaxPosition=0.16;
        armMinPosition=0.025;
        armPosition=0.03;
        gripPosition=0.95;
        guidePosition=0.55;
        gatePosition=0.40;

        robot.init(hardwareMap);

        robot.gripServo.setPosition(gripPosition);
        robot.armServo.setPosition(armPosition);
        //robot.guideServo.setPosition(guidePosition);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            if (pipeline.getAnalysis() > 155) {
                telemetry.addLine("four");
                //move of up to launc line to shoot
                encoderDrive(DRIVE_SPEED,45,45,45,45,10);

                sleep(1000);

                //turn left to shoot rings
                encoderDrive(0.2,-3,3,-3,3,10);
                sleep(1000);

                robot.leftShooter.setPower(1);
                sleep(1500);
                robot.gateServo.setPosition(0.4);
                robot.conveyor.setPower(1);
                robot.intake.setPower(1);
                sleep(5000);

                robot.leftShooter.setPower(0);
                robot.conveyor.setPower(0);
                robot.intake.setPower(0);

                sleep(1000);

                //turn right to straighten out
                encoderDrive(0.2,3,-3,3,-3,10);

                sleep(1000);

                //move forward to be perpindicularly aligned with wobble zone A
                encoderDrive(DRIVE_SPEED,15,15,15,15,10);

                //turn right so the back of the robot is parrelell with the starting wall
                encoderDrive(0.2,15,-15,15,-15,10);


                //back into wobble zone A(not needed for blue left)
                //encoderDrive(DRIVE_SPEED,-10,-10,-10,-10,10);

                //get the arm and gripper to drop the wobble
                armPosition = armMaxPosition;
                gripPosition = 0.55;
                robot.armServo.setPosition(armPosition);
                sleep(2500);
                robot.gripServo.setPosition(gripPosition);
                sleep(2000);

                //move forward out of wobble zone
                encoderDrive(DRIVE_SPEED,2,2,2,2,10);

                //reset the arm and gripper to starting position
                armPosition = armMinPosition;
                gripPosition = 0.95;
                robot.armServo.setPosition(armPosition);
                robot.gripServo.setPosition(gripPosition);

                //turn left so the robot is perpindicualer with the starting wall
                encoderDrive(0.2,-15,15,-15,15,10);
                break;
            } else if (pipeline.getAnalysis() > 140) {
                telemetry.addLine("One");
                //move of up to launc line to shoot
                encoderDrive(DRIVE_SPEED,45,45,45,45,10);

                sleep(1000);

                //turn left to shoot rings
                encoderDrive(0.2,-3,3,-3,3,10);
                sleep(1000);

                robot.leftShooter.setPower(1);
                sleep(1500);
                robot.gateServo.setPosition(0.4);
                robot.conveyor.setPower(1);
                robot.intake.setPower(1);
                sleep(5000);

                robot.leftShooter.setPower(0);
                robot.conveyor.setPower(0);
                robot.intake.setPower(0);

                sleep(1000);

                //turn right to straighten out
                encoderDrive(0.2,3,-3,3,-3,10);

                sleep(1000);

                //move forward to be perpindicularly aligned with wobble zone B
                encoderDrive(DRIVE_SPEED,25,25,25,25,10);

                //turn left so the back of the robot is parrelell with the starting wall
                encoderDrive(0.2,-15,15,-15,15,10);


                //back into wobble zone A(not needed for blue left)
                //encoderDrive(DRIVE_SPEED,-10,-10,-10,-10,10);

                //get the arm and gripper to drop the wobble
                armPosition = armMaxPosition;
                gripPosition = 0.55;
                robot.armServo.setPosition(armPosition);
                sleep(2500);
                robot.gripServo.setPosition(gripPosition);
                sleep(2000);

                //move forward out of wobble zone
                encoderDrive(DRIVE_SPEED,2,2,2,2,10);

                //reset the arm and gripper to starting position
                armPosition = armMinPosition;
                gripPosition = 0.95;
                robot.armServo.setPosition(armPosition);
                robot.gripServo.setPosition(gripPosition);

                //turn right so the robot is perpindicualer with the starting wall
                encoderDrive(0.2,15,-15,15,-15,10);

                //back into launch line
                encoderDrive(DRIVE_SPEED,-10,-10,-10,-10,10);
                break;
            } else {
                telemetry.addLine("none");
                //move of up to launc line to shoot
                encoderDrive(DRIVE_SPEED,47,47,47,47,10);

                sleep(1000);

                //turn left to shoot rings
                encoderDrive(0.2,-3,3,-3,3,10);
                sleep(1000);

                robot.leftShooter.setPower(1);
                sleep(1500);
                robot.gateServo.setPosition(0.4);
                robot.conveyor.setPower(1);
                robot.intake.setPower(1);
                sleep(5000);

                robot.leftShooter.setPower(0);
                robot.conveyor.setPower(0);
                robot.intake.setPower(0);

                sleep(1000);

                //turn right to straighten out
                encoderDrive(0.2,3,-3,3,-3,10);

                sleep(1000);

                //move forward to be perpindicularly aligned with wobble zone A
                encoderDrive(DRIVE_SPEED,20,20,20,20,10);

                //turn right so the back of the robot is parrelell with the starting wall
                encoderDrive(0.2,15,-15,15,-15,10);


                //back into wobble zone A(not needed for blue left)
                //encoderDrive(DRIVE_SPEED,-10,-10,-10,-10,10);

                //get the arm and gripper to drop the wobble
                armPosition = armMaxPosition;
                gripPosition = 0.55;
                robot.armServo.setPosition(armPosition);
                sleep(2500);
                robot.gripServo.setPosition(gripPosition);
                sleep(2000);

                //move forward out of wobble zone
                encoderDrive(DRIVE_SPEED,2,2,2,2,10);

                //reset the arm and gripper to starting position
                armPosition = armMinPosition;
                gripPosition = 0.95;
                robot.armServo.setPosition(armPosition);
                robot.gripServo.setPosition(gripPosition);

                //turn left so the robot is perpindicualer with the starting wall
                encoderDrive(0.2,-15,15,-15,15,10);

                //back into launch line
                encoderDrive(DRIVE_SPEED,-20,-20,-20,-20,10);
                break;
            }
            // Don't burn CPU cycles busy-looping in this sample
            //sleep(50);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(270,130);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    public void encoderDrive(double speed,
                             double frontleftInches, double frontrightInches,
                             double backleftInches, double backrightInches,
                             double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = (int)(frontleftInches * COUNTS_PER_INCH);
            newfrontRightTarget = (int)(frontrightInches * COUNTS_PER_INCH);
            newbackLeftTarget = (int)(backleftInches * COUNTS_PER_INCH);
            newbackRightTarget = (int)(backrightInches * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newfrontLeftTarget);
            robot.frontRight.setTargetPosition(newfrontRightTarget);
            robot.backLeft.setTargetPosition(newbackLeftTarget);
            robot.backRight.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setVelocity(Math.abs(speed));
            robot.frontRight.setVelocity(Math.abs(speed));
            robot.backLeft.setVelocity(Math.abs(speed));
            robot.backRight.setVelocity(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
                            robot.backRight.isBusy() && robot.backLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontLeftTarget,  newfrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setVelocity(0);
            robot.frontRight.setVelocity(0);
            robot.backLeft.setVelocity(0);
            robot.backRight.setVelocity(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}