package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Optional;
import java.util.Scanner;
import java.util.function.Consumer;

public class BuildInfo {
  /**
   * Exception type thrown to indicate an invalid build info file format. This exception is intended
   * for use in flow control and should be caught (and ignored) to avoid crashing robot code.
   */
  private static class InvalidFormatException extends RuntimeException {}

  /**
   * Loads the build info from the file in the deploy directory.
   *
   * @return Optional containing the build info if the file could be loaded, otherwise empty
   */
  private static Optional<String> loadBuildInfo() {
    try {
      File buildInfoFile = new File(Filesystem.getDeployDirectory(), "build-info.txt");
      return Optional.of(Files.readString(buildInfoFile.toPath()));
    } catch (IOException e) {
      DriverStation.reportError("Could not open build info file!", true);
      return Optional.empty();
    }
  }

  /**
   * Checks whether the line is a header line with the given header.
   *
   * @param line The line to check.
   * @param header The header to check against (without the parentheses).
   * @return True if the line is a header line with the given header, otherwise False.
   */
  private static boolean lineIsHeader(String line, String header) {
    return line.equals("### " + header) || line.startsWith("### " + header + " (");
  }

  /**
   * Ensures the line is a header line with the given header. If it is not, reports an error to the
   * Driver Station and throws a {@link InvalidFormatException}. That exception is intended only for
   * flow control and should be caught outside the processing code.
   *
   * @param line The line to check.
   * @param header The header to check against (without the parentheses).
   */
  private static void expectLineIsHeader(String line, String header) {
    if (!lineIsHeader(line, header)) {
      DriverStation.reportError(
          "Invalid build info file! Expected header \"" + header + "\", got line \"" + line + "\"",
          true);
      throw new InvalidFormatException();
    }
  }

  /**
   * Processes lines from the scanner up to and including the line with the given header. After this
   * function finishes, the scanner's position will be immediately after the line with the header.
   *
   * @param scanner The scanner to read lines from.
   * @param header The header to search for.
   * @param processor A lambda to process the lines as they are read.
   */
  private static void readToHeader(Scanner scanner, String header, Consumer<String> processor) {
    String line = scanner.nextLine();
    while (!lineIsHeader(line, header)) {
      processor.accept(line);
      line = scanner.nextLine();
    }
  }

  /**
   * Logs the build info from the file in the deploy directory. All build info goes to stdout, and
   * the git information is also published to NetworkTables under the BuildInfo table.
   */
  public static void logBuildInfo() {
    Optional<String> buildInfo = loadBuildInfo();
    // Print info to stdout
    System.out.println("Build info:\n" + buildInfo.orElse("N/A"));
    // Extract data from the build info
    String commitHash = "<N/A>";
    String[] commitRefs = {};
    String commitTime = "<N/A>";
    String commitMessage = "<N/A>";
    String[] remotes = {};
    String[] changedFiles = {};
    String[] untrackedFiles = {};
    if (buildInfo.isPresent()) {
      try (Scanner scanner = new Scanner(buildInfo.get())) {
        // Read to project path
        readToHeader(scanner, "Project path", (line) -> {});
        // Skip project path
        scanner.nextLine();
        // Check commit hash header
        String commitHashHeader = scanner.nextLine();
        if (!lineIsHeader(commitHashHeader, "Commit hash")) {
          System.out.println("No git!");
          throw new InvalidFormatException();
        }
        // Read the commit hash
        commitHash = scanner.nextLine();
        // Read the commit refs
        expectLineIsHeader(scanner.nextLine(), "Refs to latest commit");
        commitRefs = scanner.nextLine().split(", ");
        // Read the commit time
        expectLineIsHeader(scanner.nextLine(), "Commit time");
        commitTime = scanner.nextLine();
        // Read the commit message
        expectLineIsHeader(scanner.nextLine(), "Commit message");
        StringBuilder commitMessageBuilder = new StringBuilder();
        readToHeader(scanner, "Remotes", (line) -> commitMessageBuilder.append(line + "\n"));
        commitMessage = commitMessageBuilder.toString();
        // Read the remotes
        // Header was processed in the readToHeader() call
        ArrayList<String> remotesList = new ArrayList<>();
        readToHeader(scanner, "Changed files", (line) -> remotesList.add(line));
        remotes = remotesList.toArray(String[]::new);
        // Read the changed files
        // Header was processed in the readToHeader() call
        ArrayList<String> changedFilesList = new ArrayList<>();
        readToHeader(scanner, "Untracked files", (line) -> changedFilesList.add(line));
        changedFiles = changedFilesList.toArray(String[]::new);
        // Read the untracked files
        // Header was processed in the readToHeader() call
        ArrayList<String> untrackedFilesList = new ArrayList<>();
        readToHeader(scanner, "(END)", (line) -> untrackedFilesList.add(line));
        untrackedFiles = untrackedFilesList.toArray(String[]::new);
      } catch (InvalidFormatException e) {
        // This exception is just for control flow
      }
    }
    // Log the extracted data
    NetworkTable table = NetworkTableInstance.getDefault().getTable("BuildInfo");
    table.getStringTopic("Commit hash").publish().set(commitHash);
    table.getStringArrayTopic("References to commit").publish().set(commitRefs);
    table.getStringTopic("Commit time").publish().set(commitTime);
    table.getStringTopic("Commit message").publish().set(commitMessage);
    table.getStringArrayTopic("Remotes").publish().set(remotes);
    table.getStringArrayTopic("Changed files").publish().set(changedFiles);
    table.getStringArrayTopic("Untracked files").publish().set(untrackedFiles);
  }
}
