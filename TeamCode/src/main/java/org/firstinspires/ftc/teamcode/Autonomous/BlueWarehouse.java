package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Enums.PatrickState;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.Felipe;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Test")
@Disabled
public class BlueWarehouse extends LinearOpMode {
    public static double DISTANCE = 30; // in
    Felipe felipe = new Felipe(this); // instantiate Felipe (the main implement)
    CarouselTurnerThingy carousel = new CarouselTurnerThingy();
    // init and setup
    ElapsedTime runtime = new ElapsedTime();


    // ENUMS
    //DriveSpeedState  currDriveState;
    PatrickState patrickState = PatrickState.OFF;
    LiftPosition liftPosition = LiftPosition.LOAD;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ElapsedTime timer = new ElapsedTime();
        double ducktime = 4; // carosel rotation time
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap);

        ///////////////////////////////////////////////////////////////////////////
        // Trajectories Here
        ///////////////////////////////////////////////////////////////////////////

        // The short distances appear to make the more complex motions fail. Thus this is done step by step
        Trajectory  traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .addTemporalMarker(-1,()->{felipe.highGoalRight();})
                .build();
        // note that trajectory 2 has a start pose that is the end of trajectory 1 plus the 90 deg rotation from the
        // drive.turn command between traj 1 and traj 2.
        Trajectory  traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .back(20)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(8)
                .addTemporalMarker(-0.3,()->{felipe.homieLeft();})
                .build();
        // Move away from the alliance shipping hub so the arm can be retracted without hitting the hub
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(9)
                .addDisplacementMarker(()->{felipe.reset();})
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToConstantHeading(new Vector2d(-7 ,10), Math.toRadians(90))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(28)
                .build();

//made mmment


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);

        // dealy to let acrosel turn






        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
