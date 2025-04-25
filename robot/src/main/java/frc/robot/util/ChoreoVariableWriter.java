/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.util;

import static frc.robot.constants.FieldConstants.*;

import com.google.gson.*;
import com.google.gson.stream.JsonWriter;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.System.ArmState;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.HashMap;
import java.util.Map;

public final class ChoreoVariableWriter {

    public static void writeToChoreo() {
        String filePath = "robot/src/main/deploy/choreo/test.chor"; // Path to your JSON file

        Map<String, Pose2d> namedPosesMap = new HashMap<>();

        String[] redRightNames = {"REEF_RED_A", "REEF_RED_C", "REEF_RED_E", "REEF_RED_G", "REEF_RED_I", "REEF_RED_K"};
        for (int i = 0; i < REEF_RED_RIGHT.length; i++) {
            namedPosesMap.put(redRightNames[i], applyL4Offset(REEF_RED_RIGHT[i], false));
        }
        String[] redLeftNames = {"REEF_RED_B", "REEF_RED_D", "REEF_RED_F", "REEF_RED_H", "REEF_RED_J", "REEF_RED_L"};
        for (int i = 0; i < REEF_RED_LEFT.length; i++) {
            namedPosesMap.put(redLeftNames[i], applyL4Offset(REEF_RED_LEFT[i], true));
        }
        String[] blueRightNames = {
            "REEF_BLUE_A", "REEF_BLUE_C", "REEF_BLUE_E", "REEF_BLUE_G", "REEF_BLUE_I", "REEF_BLUE_K"
        };
        for (int i = 0; i < REEF_BLUE_RIGHT.length; i++) {
            namedPosesMap.put(blueRightNames[i], applyL4Offset(REEF_BLUE_RIGHT[i], false));
        }
        String[] blueLeftNames = {
            "REEF_BLUE_B", "REEF_BLUE_D", "REEF_BLUE_F", "REEF_BLUE_H", "REEF_BLUE_J", "REEF_BLUE_L"
        };
        for (int i = 0; i < REEF_BLUE_LEFT.length; i++) {
            namedPosesMap.put(blueLeftNames[i], applyL4Offset(REEF_BLUE_LEFT[i], true));
        }

        try (FileReader reader = new FileReader(filePath)) {
            System.out.println("[ChoreoVariableWriter]: " + filePath + " read successfully.");
            // Parse the JSON content into a JsonObject
            JsonObject jsonObject = JsonParser.parseReader(reader).getAsJsonObject();
            namedPosesMap.forEach((name, pose) -> {
                jsonObject
                        .get("variables")
                        .getAsJsonObject()
                        .get("poses")
                        .getAsJsonObject()
                        .add(name, generateJsonPoseElement(pose));
            });

            reader.close();

            Gson gson = new GsonBuilder().setPrettyPrinting().create();

            try (FileWriter writer = new FileWriter(filePath)) {
                JsonWriter jsonWriter = new JsonWriter(writer);
                gson.toJson(jsonObject, jsonWriter);
                System.out.println("[ChoreoVariableWriter]: " + filePath + " updated successfully.");
            } catch (Exception e) {
                System.err.println("Error: Failed writing " + filePath);
            }
        } catch (Exception e) {
            System.err.println("Error: Failed reading or parsing " + filePath);
        }
    }

    private static JsonElement generateJsonPoseElement(Pose2d pose) {
        return JsonParser.parseString("{" + generateExpression("x", pose.getX(), "m")
                + "," + generateExpression("y", pose.getY(), "m")
                + "," + generateExpression("heading", pose.getRotation().getRadians(), "rad")
                + "}");
    }

    private static String generateExpression(String name, Object value, String unitSI) {
        return "\"" + name + "\":{\"exp\":\"" + value + " " + unitSI + "\", \"val\":" + value + "}";
    }

    private static Pose2d applyL4Offset(Pose2d pose, boolean isLeft) {
        if (isLeft) {
            return pose.transformBy(REEF_OFFSET_MAP_LEFT.get(ArmState.L4));
        } else {
            return pose.transformBy(REEF_OFFSET_MAP_RIGHT.get(ArmState.L4));
        }
    }
}
