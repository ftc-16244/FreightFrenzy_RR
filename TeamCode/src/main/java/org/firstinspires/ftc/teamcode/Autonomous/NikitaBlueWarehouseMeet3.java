package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.Barcode;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.FelipeDeux;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Test")
public class NikitaBlueWarehouseMeet3 extends LinearOpMode {

    OpenCvCamera webcam;

    public static double DISTANCE = 30; // in
    public ElapsedTime   tfTime      = new ElapsedTime(); // timer for tensor flow
    FelipeDeux felipe = new FelipeDeux(this); // instantiate Felipe (the main implement)
    CarouselTurnerThingy carousel = new CarouselTurnerThingy();
    // init and setup
    ElapsedTime runtime = new ElapsedTime();
    Barcode barcode = Barcode.RIGHT; // Default target zone

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FreightFrenzyTSEPipeline_EXP detector = new FreightFrenzyTSEPipeline_EXP(telemetry, Alliance.BLUE, StartSide.WAREHOUSE);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //See EaayopenCV webcam example for details on this new streaming methods works
                // manay examples from Skystone and Ultimate goal have outdated methods that are deprecated

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ElapsedTime timer = new ElapsedTime();
        double ducktime = 2.5; // carosel rotation time
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap);
        felipe.juanMechanicalReset();

        ///////////////////////////////////////////////////////////////////////////
        // Trajectories - HIGH GOAL
        ///////////////////////////////////////////////////////////////////////////
        Trajectory  traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .build();

        Trajectory  traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .back(19)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .addTemporalMarker(0.25,()->{felipe.armMid();})
                .strafeRight(14)

                .build();
        // Move away from the alliance shipping hub so the arm can be retracted without hitting the hub
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .addTemporalMarker(-0.8,()->{felipe.thumbOpen();})
                .addTemporalMarker(1,()->{felipe.thumbClose();})
                .addTemporalMarker(1.5,()->{felipe.armInit();})
                .strafeLeft(8.5)

                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(20)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end().plus(new Pose2d(0,0,Math.toRadians(-180))))
                .strafeRight(20)
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end().plus(new Pose2d(0,0,Math.toRadians(0))))
                .forward(35)
                .build();


        waitForStart();
        felipe.liftLoad();// put here becase opmode is acitve is a condition in the method that does this
        tfTime.reset(); //  reset the TF timer

            switch(detector.getLocation()){
                case LEFT: //
                    felipe.liftRise();
                    drive.followTrajectory(traj1);
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(traj2);
                    drive.followTrajectory(traj3);
                    drive.followTrajectory(traj4);

                    drive.followTrajectory(traj5);
                    felipe.liftLoad();
                    drive.turn(Math.toRadians(180));

                    drive.followTrajectory(traj6);
                    drive.followTrajectory(traj7);

                    break;

                case CENTER: //
                    felipe.liftRise();
                    drive.followTrajectory(traj1);
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(traj2);
                    drive.followTrajectory(traj3);
                    drive.followTrajectory(traj4);

                    drive.followTrajectory(traj5);
                    felipe.liftLoad();
                    drive.turn(Math.toRadians(180));
                    drive.followTrajectory(traj6);

                    break;

                case RIGHT: //level 3 highest goal
                    felipe.liftRise();
                    drive.followTrajectory(traj1);
                    drive.turn(Math.toRadians(180));
                    drive.followTrajectory(traj2);
                    drive.followTrajectory(traj3);
                    drive.followTrajectory(traj4);

                    drive.followTrajectory(traj5);
                    felipe.liftLoad();
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(traj6);

                    break;
            }
        //}

        if (isStopRequested()) return;


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;

        webcam.stopStreaming();
    }

//checking if my github works
}
