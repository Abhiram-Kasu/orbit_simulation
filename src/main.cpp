#include "planet.hpp"
#include "raylib.h"
#include "rocket.hpp"
#include <Eigen/Dense>
#include <expected>
#include <numbers>
#include <string>

namespace {
// Window dimensions
constexpr int SCREEN_WIDTH = 1200;
constexpr int SCREEN_HEIGHT = 800;

// Physics configuration
struct PhysicsConfig {
  float gravitationalConstant = 100.0f;
  float planetMass = 1000.0f;
  float rocketMass = 1.0f;
  float thrustForce = 200.0f;
  float timeScale = 1.0f;
  float initialVelocity = 100.0f;
  float zoomSpeed = 0.1f;
  float minZoom = 0.1f;
  float maxZoom = 5.0f;
  float rotationSpeed = 2.0f;
};

PhysicsConfig config{};

// Function to reset rocket with orbital velocity
[[nodiscard]] auto resetRocket(Rocket &rocket, const Planet &planet,
                               const Eigen::Vector2f &position)
    -> std::expected<void, const char *> {
  // Calculate direction perpendicular to planet-rocket vector
  Eigen::Vector2f toRocket = position - planet.getPosition();
  float distance = toRocket.norm();

  if (distance <= planet.getRadius())
    return std::unexpected("Invalid starting position: Inside planet");

  Eigen::Vector2f perpendicular(-toRocket.y(), toRocket.x());
  perpendicular.normalize();

  // Calculate orbital velocity for approximate circular orbit
  // v = sqrt(GM/r)

  float orbitalSpeed =
      std::sqrt(config.gravitationalConstant * planet.getMass() / distance);

  // Set rocket position and velocity
  return rocket.reset(position, perpendicular * orbitalSpeed);
}

// Convert screen coordinates to world coordinates
[[nodiscard]] auto screenToWorld(const Vector2 &screenPos,
                                 const Camera2D &camera) -> Eigen::Vector2f {
  return {(screenPos.x - camera.offset.x) / camera.zoom + camera.target.x,
          (screenPos.y - camera.offset.y) / camera.zoom + camera.target.y};
}

// Format float with specified precision
[[nodiscard]] auto formatFloat(float value, int precision = 2) -> std::string {
  return TextFormat("%.*f", precision, value);
}

} // namespace

auto main() -> int {

  // Initialize window
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Orbit Simulation");
  SetTargetFPS(60);

  // Initialize camera
  Camera2D camera{
      .offset = {SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f},
      .target = {SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f},
      .rotation = 0.0f,
      .zoom = 1.0f,
  };

  // Create game objects
  Planet planet({SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f}, 40.0f,
                config.planetMass);

  // Initial rocket position
  Eigen::Vector2f initialPos(SCREEN_WIDTH / 2.0f - 200.0f,
                             SCREEN_HEIGHT / 2.0f);
  Rocket rocket(initialPos, config.rocketMass, config.thrustForce);

  if (auto result = resetRocket(rocket, planet, initialPos); !result) {
    TraceLog(LOG_ERROR, "Failed to reset rocket: %s", result.error());
    return 1;
  }

  bool showVectors = true;

  // Main game loop
  while (!WindowShouldClose()) {
    float deltaTime = GetFrameTime() * config.timeScale;

    // Camera zoom control with smooth interpolation
    if (float wheel = GetMouseWheelMove(); wheel != 0) {
      float targetZoom =
          std::clamp(camera.zoom + wheel * config.zoomSpeed * camera.zoom,
                     config.minZoom, config.maxZoom);
      camera.zoom = std::lerp(camera.zoom, targetZoom, 0.1f);
    }

    // Handle input
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
      Vector2 mouseScreenPos = GetMousePosition();
      Eigen::Vector2f mouseWorldPos = screenToWorld(mouseScreenPos, camera);
      if (auto result = resetRocket(rocket, planet, mouseWorldPos); !result) {
        TraceLog(LOG_WARNING, "Failed to reset rocket: %s", result.error());
      }
    }

    // Reset simulation
    if (IsKeyPressed(KEY_R)) {
      if (auto result = resetRocket(rocket, planet, initialPos); !result) {
        TraceLog(LOG_WARNING, "Failed to reset rocket: %s", result.error());
      }
      config.timeScale = 1.0f;
      camera.zoom = 1.0f;
      camera.target = {SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f};
      config.gravitationalConstant = 100.0f;
    }

    // Toggle vector visualization
    if (IsKeyPressed(KEY_V)) {
      showVectors = !showVectors;
    }

    // Gravity controls with smooth interpolation
    if (IsKeyDown(KEY_UP))
      config.gravitationalConstant *= (1.0f + deltaTime);
    if (IsKeyDown(KEY_DOWN))
      config.gravitationalConstant *= (1.0f - deltaTime);

    // Thrust controls with improved precision
    Eigen::Vector2f thrustDirection = Eigen::Vector2f::Zero();
    if (IsKeyDown(KEY_W))
      thrustDirection = rocket.getForwardDirection();
    if (IsKeyDown(KEY_S))
      thrustDirection = -rocket.getForwardDirection();
    if (IsKeyDown(KEY_A))
      rocket.rotate(-config.rotationSpeed * deltaTime);
    if (IsKeyDown(KEY_D))
      rocket.rotate(config.rotationSpeed * deltaTime);

    if (thrustDirection != Eigen::Vector2f::Zero()) {
      rocket.applyThrust(thrustDirection, deltaTime);
    }

    // Time scale controls with smooth transitions
    if (IsKeyPressed(KEY_RIGHT))
      config.timeScale = std::min(config.timeScale * 2.0f, 16.0f);
    if (IsKeyPressed(KEY_LEFT))
      config.timeScale = std::max(config.timeScale / 2.0f, 0.25f);
    if (IsKeyPressed(KEY_SPACE))
      config.timeScale = 1.0f;

    // Camera panning with smooth movement
    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
      Vector2 delta = GetMouseDelta();
      camera.target.x -= delta.x / camera.zoom;
      camera.target.y -= delta.y / camera.zoom;
    }

    // Calculate gravitational force
    auto gravityResult = planet.calculateGravitationalForce(
        rocket.getPosition(), rocket.getMass(), config.gravitationalConstant);
    if (gravityResult) {
      // Update physics
      rocket.applyForce(gravityResult->force, deltaTime);
      rocket.update(deltaTime);
    } else {
      TraceLog(LOG_WARNING, "Gravity calculation failed: %s",
               gravityResult.error());
    }

    // Drawing
    BeginDrawing();
    ClearBackground(BLACK);

    BeginMode2D(camera);

    // Draw objects
    planet.draw(camera);
    rocket.draw(camera, gravityResult->force, showVectors);

    EndMode2D();

    // Draw UI with improved formatting
    constexpr int textSize = 20;
    constexpr int startX = 10;
    int currentY = 10;
    const int lineSpacing = textSize + 5;

    // Status information
    DrawText(
        TextFormat("Time Scale: %sx", formatFloat(config.timeScale).c_str()),
        startX, currentY, textSize, WHITE);
    currentY += lineSpacing;

    DrawText(TextFormat("Velocity: %s",
                        formatFloat(rocket.getVelocityMagnitude()).c_str()),
             startX, currentY, textSize, WHITE);
    currentY += lineSpacing;

    DrawText(TextFormat("Gravity: %s",
                        formatFloat(config.gravitationalConstant).c_str()),
             startX, currentY, textSize, WHITE);
    currentY += lineSpacing;

    DrawText(TextFormat("Zoom: %sx", formatFloat(camera.zoom).c_str()), startX,
             currentY, textSize, WHITE);

    // Help text
    constexpr int helpStartY = SCREEN_HEIGHT - 180;
    currentY = helpStartY;

    DrawText("Controls:", startX, currentY, textSize, WHITE);
    currentY += lineSpacing;

    DrawText("W/S: Forward/Reverse Thrust", startX, currentY, textSize, WHITE);
    currentY += lineSpacing;

    DrawText("A/D: Rotate Rocket", startX, currentY, textSize, WHITE);
    currentY += lineSpacing;

    DrawText("UP/DOWN: Adjust Gravity", startX, currentY, textSize, WHITE);
    currentY += lineSpacing;

    DrawText("Mouse Wheel: Zoom   Middle Mouse: Pan", startX, currentY,
             textSize, WHITE);
    currentY += lineSpacing;

    DrawText("LEFT/RIGHT: Time Scale   SPACE: Reset Time", startX, currentY,
             textSize, WHITE);
    currentY += lineSpacing;

    DrawText("R: Reset Simulation   V: Toggle Vectors", startX, currentY,
             textSize, WHITE);

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
