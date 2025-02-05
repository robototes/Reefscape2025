package frc.robot.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.parser.ParseException;

public class PathPlannerAutos {
  public static final class Auto {
    public final Pose2d startingPose;
    public final List<PathPlannerTrajectory> trajectories;

    public Auto() {
      this.startingPose = null;
      this.trajectories = List.of();
    }

    public Auto(Pose2d startingPose, List<PathPlannerTrajectory> trajectories) {
      this.startingPose = startingPose;
      this.trajectories = trajectories;
    }
  }

  // Registering named commands

  private static final Map<String, List<PathPlannerPath>> namedCommandPathsCache = new HashMap<>();

  // Individual paths

  private static PathPlannerPath loadPathPlannerPath(String name) {
    try {

      return PathPlannerPath.fromPathFile(name);
    } catch (IOException | ParseException | FileVersionException e) {
      return null;
    }
  }

  public static void registerAuto(String autoName, List<PathPlannerPath> paths) {
    namedCommandPathsCache.put(autoName, paths);
  }

  public static void registerAuto(String autoName, String... pathNames) {
    List<PathPlannerPath> paths = new ArrayList<>(pathNames.length);
    for (String pathName : pathNames) {
      paths.add(getPathPlannerPath(pathName));
    }
    registerAuto(autoName, paths);
  }

  private static final Map<String, PathPlannerPath> pathPlannerPathCache = new HashMap<>();
  private static final Map<String, PathPlannerPath> choreoTrajectoryCache = new HashMap<>();

  public static PathPlannerPath getPathPlannerPath(String name) {
    return pathPlannerPathCache.computeIfAbsent(name, PathPlannerAutos::loadPathPlannerPath);
  }

  // Whole autos

  private static void addPathsFromCommand(
      JsonNode commandJson, boolean isChoreo, List<PathPlannerPath> paths) {
    String type = commandJson.get("type").asText();
    switch (type) {
      case "wait":
        break;
      case "named":
        String commandName = commandJson.get("data").get("name").asText();
        paths.addAll(namedCommandPathsCache.getOrDefault(commandName, List.of()));
        break;
      case "path":
        String pathName = commandJson.get("data").get("pathName").asText();
        paths.add(getPathPlannerPath(pathName));
        break;
      case "sequential", "parallel", "race", "deadline":
        for (JsonNode child : commandJson.get("data").get("commands")) {
          addPathsFromCommand(child, isChoreo, paths);
        }
        break;
    }
  }

  private static List<PathPlannerPath> getPaths(JsonNode autoJson) {
    JsonNode commandRoot = autoJson.get("command");
    boolean isChoreo = autoJson.get("choreoAuto").asBoolean();
    List<PathPlannerPath> paths = new ArrayList<>();
    addPathsFromCommand(commandRoot, isChoreo, paths);
    return paths;
  }

  /*private static ChassisSpeeds speedsFromState(SwerveModuleTrajectoryState state) {
      return new ChassisSpeeds(
          state.heading.getCos() * state.linearVelocity,

          state.heading.getSin() * state.velocityMps,
          state.headingAngularVelocityRps);
    }


    private static List<PathPlannerTrajectory> trajectoriesFromPaths(
        List<PathPlannerPath> paths, Rotation2d startingRotation) {
      if (paths.isEmpty()) {
        return List.of();
      }
      if (startingRotation == null) {
        startingRotation = paths.get(0).getPreviewStartingHolonomicPose().getRotation();
      }
      ChassisSpeeds startingSpeeds = new ChassisSpeeds();
      List<PathPlannerTrajectory> trajectories = new ArrayList<>(paths.size());
      for (var path : paths) {
        PathPlannerTrajectory trajectory = path.getTrajectory(startingSpeeds, startingRotation);
        trajectories.add(trajectory);
        startingRotation = trajectory.getEndState().targetHolonomicRotation;
        startingSpeeds = speedsFromState(trajectory.getEndState());
      }
      return List.copyOf(trajectories);
    }

    private static Pose2d getStartingPose(JsonNode autoJson) {
      JsonNode startingPose = autoJson.get("startingPose");
      if (startingPose.isNull()) {
        return null;
      }
      JsonNode translation = startingPose.get("position");
      return new Pose2d(
          translation.get("x").asDouble(),
          translation.get("y").asDouble(),
          Rotation2d.fromDegrees(startingPose.get("rotation").asDouble()));
    }

    private static Auto loadAuto(String autoName) {
      File autoFile =
          new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto");
      if (!autoFile.exists()) {
        DriverStation.reportWarning(
            "Attempted to load non-existent auto \"" + autoName + "\"", false);
        return new Auto();
      }
      JsonNode autoJson;
      try {
        autoJson = new ObjectMapper().readTree(autoFile);
      } catch (IOException e) {
        DriverStation.reportWarning("Could not load auto \"" + autoName + "\"", e.getStackTrace());
        return new Auto();
      }
      Pose2d startingPose = getStartingPose(autoJson);
      Rotation2d startingRotation = startingPose == null ? null : startingPose.getRotation();
      List<PathPlannerTrajectory> trajectories =
          trajectoriesFromPaths(getPaths(autoJson), startingRotation);
      return new Auto(startingPose, trajectories);
    }

    private static final Map<String, Auto> autosCache = new HashMap<>();

    public static Auto getAuto(String autoName) {
      return autosCache.computeIfAbsent(autoName, PathPlannerAutos::loadAuto);
    }

    // Pre-loading everything

    private static List<String> listFileNames(String deploySubdirectory, String extension) {
      File[] files = new File(Filesystem.getDeployDirectory(), deploySubdirectory).listFiles();

      if (files == null) {
        return List.of();
      }

      List<String> filenames = new ArrayList<>(files.length);
      for (File file : files) {
        if (!file.isDirectory()) {
          String name = file.getName();
          if (name.endsWith(extension)) {
            filenames.add(name.substring(0, name.lastIndexOf(".")));
          }
        }
      }
      return filenames;
    }

    private static <T> void loadAll(
        String deploySubdirectory,
        String extension,
        Map<String, T> cache,
        Function<String, T> loader) {
      List<String> names = listFileNames(deploySubdirectory, extension);
      for (String name : names) {
        cache.put(name, loader.apply(name));
      }
    }

    public static void loadAllPathPlannerPaths() {
      loadAll(
          "pathplanner/paths", ".path", pathPlannerPathCache, PathPlannerAutos::loadPathPlannerPath);
    }



    public static void loadAllAutoTrajectories() {
      loadAll("pathplanner/autos", ".auto", autosCache, PathPlannerAutos::loadAuto);
    }

  }
    */

}
