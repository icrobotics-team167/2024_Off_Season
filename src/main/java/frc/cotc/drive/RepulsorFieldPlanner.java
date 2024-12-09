// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.cotc.Constants;
import frc.cotc.Robot;
import java.util.ArrayList;
import java.util.List;

public class RepulsorFieldPlanner {
  abstract static class Obstacle {
    double strength;
    boolean positive;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    public abstract Translation2d getForceAtPosition(Translation2d position, Translation2d target);

    protected double getForceAtDistance(double distanceMeters) {
      if (MathUtil.isNear(0, distanceMeters, 1e-2)) {
        distanceMeters = 1e-2;
      }
      var force = strength / (distanceMeters * distanceMeters);
      return positive ? force : -force;
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d location;
    double radiusMeters = .5;

    public PointObstacle(Translation2d location, double strength) {
      super(strength, true);
      this.location = location;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var distanceToCurrentPos = location.getDistance(position);
      if (distanceToCurrentPos > 4) {
        return Translation2d.kZero;
      }

      var outwardsForceMagnitude = getForceAtDistance(distanceToCurrentPos - radiusMeters);
      var initialForce =
          new Translation2d(outwardsForceMagnitude, position.minus(location).getAngle());

      var theta = target.minus(position).getAngle().minus(position.minus(location).getAngle());
      var sidewaysMagnitude =
          outwardsForceMagnitude * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      var sidewaysForce =
          initialForce
              .div(initialForce.getNorm()) // Normalize
              .times(sidewaysMagnitude) // Set new vector length
              .rotateBy(Rotation2d.kCCW_90deg); // Make perpendicular to original

      return initialForce.plus(sidewaysForce);
    }
  }

  static class SnowmanObstacle extends Obstacle {
    Translation2d location;
    double radiusMeters = .5;

    public SnowmanObstacle(Translation2d location, double strength) {
      super(strength, true);
      this.location = location;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var locationToTargetDelta = location.minus(target);
      var deltaAngle = locationToTargetDelta.getAngle();

      var sidewaysForceCenter = new Translation2d(1, deltaAngle).plus(location);

      var currentToTargetDistance = location.getDistance(position);
      var sidewaysForceDistance = sidewaysForceCenter.getDistance(position);
      if (currentToTargetDistance > 2 && sidewaysForceDistance > 2) {
        return Translation2d.kZero;
      }

      var outwardsForceMag = getForceAtDistance(location.getDistance(position));
      var sidewaysForceMagnitude =
          getForceAtDistance(sidewaysForceCenter.getDistance(position)) / 2;

      var outwardsForce = new Translation2d(outwardsForceMag, position.minus(location).getAngle());

      // flip sidewaysForce based on which side the robot is on
      var sidewaysTheta =
          target.minus(position).getAngle().minus(position.minus(sidewaysForceCenter).getAngle());

      double sidewaysForce =
          sidewaysForceMagnitude * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      return outwardsForce.plus(
          new Translation2d(sidewaysForce, deltaAngle.rotateBy(Rotation2d.kCCW_90deg)));
    }
  }

  static class HorizontalObstacle extends Obstacle {
    double y;

    public HorizontalObstacle(double y, double strength, boolean positive) {
      super(strength, positive);
      this.y = y;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var distance = Math.abs(position.getX() - y);
      if (distance > 1) {
        return Translation2d.kZero;
      }
      return new Translation2d(0, getForceAtDistance(y - position.getX()));
    }
  }

  static class VerticalObstacle extends Obstacle {
    double x;

    public VerticalObstacle(double x, double strength, boolean positive) {
      super(strength, positive);
      this.x = x;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var distance = Math.abs(position.getX() - x);
      if (distance > 1) {
        return Translation2d.kZero;
      }
      return new Translation2d(getForceAtDistance(x - position.getX()), 0);
    }
  }

  public static final double GOAL_STRENGTH = 1;

  private static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new SnowmanObstacle(new Translation2d(5.56, 2.74), 0.4),
          new SnowmanObstacle(new Translation2d(3.45, 4.07), 0.4),
          new SnowmanObstacle(new Translation2d(5.56, 5.35), 0.4),
          new SnowmanObstacle(new Translation2d(11.0, 2.74), 0.4),
          new SnowmanObstacle(new Translation2d(13.27, 4.07), 0.4),
          new SnowmanObstacle(new Translation2d(11.0, 5.35), 0.4));

  private static final List<Obstacle> WALLS =
      List.of(
          new HorizontalObstacle(0, .5, true),
          new HorizontalObstacle(Constants.FIELD_WIDTH, .5, false),
          new VerticalObstacle(0, .5, true),
          new VerticalObstacle(Constants.FIELD_LENGTH, .5, false));

  private static final List<Obstacle> STATIC_OBSTACLES;

  static {
    STATIC_OBSTACLES = new ArrayList<>();
    STATIC_OBSTACLES.addAll(FIELD_OBSTACLES);
    STATIC_OBSTACLES.addAll(WALLS);
  }

  public SwerveSample run(
      Translation2d target, Pose2d currentPose, ChassisSpeeds currentSpeeds, double maxSpeed) {
    var velocityVector =
        new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    var currentSpeed = velocityVector.getNorm();

    var stepSize = maxSpeed * Robot.defaultPeriodSecs;

    var currentTranslation = currentPose.getTranslation();
    var error = target.minus(currentTranslation);

    if (error.getNorm() < stepSize * 1.5) {
      return sample(target, target.getAngle(), 0, 0);
    }

    var obstacleForce = Translation2d.kZero;
    for (Obstacle obs : STATIC_OBSTACLES) {
      obstacleForce = obstacleForce.plus(obs.getForceAtPosition(currentTranslation, target));
    }

    Translation2d goalForce;
    if (MathUtil.isNear(0, error.getNorm(), 1e-5)) {
      goalForce = Translation2d.kZero;
    } else {
      var mag = GOAL_STRENGTH * (1 / (error.getNorm()));
      goalForce = new Translation2d(mag, error.getAngle());
    }

    var netForce = goalForce.plus(obstacleForce);

    var closeToGoalMax = maxSpeed * Math.min(error.getNorm() / 2, 1);
    stepSize = Math.min(maxSpeed, closeToGoalMax) * Robot.defaultPeriodSecs;
    var step = new Translation2d(stepSize, netForce.getAngle());
    var intermediateGoal = currentTranslation.plus(step);
    return sample(
        intermediateGoal,
        target.getAngle(),
        step.getX() / Robot.defaultPeriodSecs,
        step.getY() / Robot.defaultPeriodSecs);
  }

  private SwerveSample sample(Translation2d trans, Rotation2d rot, double vx, double vy) {
    return new SwerveSample(
        0,
        trans.getX(),
        trans.getY(),
        rot.getRadians(),
        vx,
        vy,
        0,
        0,
        0,
        0,
        new double[4],
        new double[4]);
  }
}
