#ifndef ROCKET_HPP
#define ROCKET_HPP

#include "raylib.h"
#include <Eigen/Dense>
#include <concepts>
#include <expected>
#include <numbers>

class Rocket {
private:
  Eigen::Vector2f position;
  Eigen::Vector2f velocity;
  Eigen::Vector2f acceleration;
  float mass;
  float thrust;
  float rotation; // in radians
  float size;     // visual size for rendering
  Color bodyColor;
  Color thrustColor;
  Color outlineColor;
  bool isThrusting;

  // Helper function to scale vector visualization based on magnitude
  [[nodiscard]] static constexpr auto scaleVectorSize(float magnitude)
      -> float {
    constexpr float minScale = 0.5f;
    constexpr float maxScale = 5.0f;
    constexpr float scaleFactor = 0.01f;
    return std::clamp(magnitude * scaleFactor, minScale, maxScale);
  }

  // Helper function to draw vector arrows with dynamic scaling
  static auto drawVectorArrow(const Vector2 &start, const Vector2 &end,
                              float magnitude, float baseSize, Color color)
      -> void {
    float scale = scaleVectorSize(magnitude);
    float scaledSize = baseSize * scale;

    // Draw main line with thickness based on magnitude
    DrawLineEx(start, end, 2.0f * scale, color);

    // Calculate arrow angle
    float angle = std::atan2(end.y - start.y, end.x - start.x);
    constexpr float arrowAngle = 0.5f; // ~30 degrees in radians

    // Draw arrow head with scaled size
    Vector2 arrowLeft = {end.x - scaledSize * std::cos(angle + arrowAngle),
                         end.y - scaledSize * std::sin(angle + arrowAngle)};
    Vector2 arrowRight = {end.x - scaledSize * std::cos(angle - arrowAngle),
                          end.y - scaledSize * std::sin(angle - arrowAngle)};

    DrawLineEx(end, arrowLeft, 2.0f * scale, color);
    DrawLineEx(end, arrowRight, 2.0f * scale, color);
  }

public:
  constexpr Rocket(const Eigen::Vector2f &pos, float m, float t) noexcept
      : position(pos), velocity(Eigen::Vector2f::Zero()),
        acceleration(Eigen::Vector2f::Zero()), mass(m), thrust(t), rotation(0),
        size(15.0f), bodyColor(RED), thrustColor(ORANGE), outlineColor(WHITE),
        isThrusting(false) {}

  auto reset(const Eigen::Vector2f &pos, const Eigen::Vector2f &vel)
      -> std::expected<void, const char *> {
    if (pos.hasNaN() || vel.hasNaN())
      return std::unexpected("Invalid position or velocity vector");

    position = pos;
    velocity = vel;
    acceleration = Eigen::Vector2f::Zero();
    rotation = std::atan2(vel.y(), vel.x());
    isThrusting = false;
    return {};
  }

  [[nodiscard]] auto getForwardDirection() const -> Eigen::Vector2f {
    return {std::cos(rotation), std::sin(rotation)};
  }

  auto rotate(float angle) -> void {
    rotation = std::fmod(rotation + angle, 2 * std::numbers::pi_v<float>);
  }

  auto update(float deltaTime) -> void {
    if (deltaTime <= 0)
      return;

    velocity += acceleration * deltaTime;
    position += velocity * deltaTime;
    acceleration = Eigen::Vector2f::Zero();
    isThrusting = false;
  }

  auto applyForce(const Eigen::Vector2f &force, float deltaTime) -> void {
    if (deltaTime <= 0 || force.hasNaN())
      return;
    acceleration += force / mass;
  }

  auto applyThrust(const Eigen::Vector2f &direction, float deltaTime) -> void {
    if (deltaTime <= 0 || direction.hasNaN())
      return;

    Eigen::Vector2f thrustForce = direction.normalized() * thrust;
    applyForce(thrustForce, deltaTime);
    isThrusting = true;
  }

  auto draw(const Camera2D &camera, const Eigen::Vector2f &gravityForce,
            bool showVectors) const -> void {
    // Convert world position to screen position
    Vector2 screenPos =
        GetWorldToScreen2D({position.x(), position.y()}, camera);
    float scaledSize = size * camera.zoom;

    // Calculate rocket shape points with rotation
    float cos_rot = std::cos(rotation);
    float sin_rot = std::sin(rotation);

    constexpr float tipMultiplier = 2.0f;
    constexpr float wingAngle = 2.8f;

    Vector2 tip = {screenPos.x + scaledSize * tipMultiplier * cos_rot,
                   screenPos.y + scaledSize * tipMultiplier * sin_rot};

    Vector2 left = {screenPos.x + scaledSize * std::cos(rotation + wingAngle),
                    screenPos.y + scaledSize * std::sin(rotation + wingAngle)};

    Vector2 right = {screenPos.x + scaledSize * std::cos(rotation - wingAngle),
                     screenPos.y + scaledSize * std::sin(rotation - wingAngle)};

    // Draw force vectors with dynamic scaling if enabled
    if (showVectors) {
      // Velocity vector
      if (float velMag = velocity.norm(); velMag > 0) {
        Vector2 velEnd = {
            screenPos.x + velocity.normalized().x() * scaledSize * 3,
            screenPos.y + velocity.normalized().y() * scaledSize * 3};
        drawVectorArrow(screenPos, velEnd, velMag, scaledSize, BLUE);
      }

      // Gravity force vector
      if (float gravMag = gravityForce.norm(); gravMag > 0) {
        Vector2 gravEnd = {
            screenPos.x + gravityForce.normalized().x() * scaledSize * 3,
            screenPos.y + gravityForce.normalized().y() * scaledSize * 3};
        drawVectorArrow(screenPos, gravEnd, gravMag, scaledSize, YELLOW);
      }
    }

    // Draw rocket body with outline glow effect
    DrawTriangleLines(tip, left, right, ColorAlpha(outlineColor, 0.5f));
    DrawTriangle(tip, left, right, bodyColor);

    // Draw thrust effects when thrusting
    if (isThrusting) {
      // Thrust origin point
      Vector2 thrustPoint = {screenPos.x - cos_rot * scaledSize * 1.2f,
                             screenPos.y - sin_rot * scaledSize * 1.2f};

      // Draw engine block
      float engineSize = scaledSize * 0.4f;
      Vector2 enginePos = {thrustPoint.x - engineSize,
                           thrustPoint.y - engineSize};
      DrawRectangle(enginePos.x, enginePos.y, engineSize * 2, engineSize * 2,
                    RED);

      // Draw dynamic thrust flames
      constexpr int numFlames = 3;
      for (int i = -1; i <= 1; i++) {
        float spread = i * 0.2f;
        float flameLength =
            scaledSize *
            (1.5f + static_cast<float>(GetRandomValue(0, 20)) / 40.0f);
        Vector2 flameEnd = {thrustPoint.x - cos_rot * flameLength +
                                sin_rot * spread * scaledSize,
                            thrustPoint.y - sin_rot * flameLength -
                                cos_rot * spread * scaledSize};
        DrawLineEx(thrustPoint, flameEnd, 2.0f,
                   ColorAlpha(thrustColor,
                              0.7f + static_cast<float>(GetRandomValue(0, 30)) /
                                         100.0f));
      }
    }

    // Draw direction indicator
    Vector2 directionEnd = {screenPos.x + cos_rot * scaledSize,
                            screenPos.y + sin_rot * scaledSize};
    DrawLineEx(screenPos, directionEnd, 1.0f, GREEN);
  }

  // Getters - all marked [[nodiscard]]
  [[nodiscard]] auto getPosition() const -> const Eigen::Vector2f & {
    return position;
  }
  [[nodiscard]] auto getVelocity() const -> const Eigen::Vector2f & {
    return velocity;
  }
  [[nodiscard]] auto getVelocityMagnitude() const -> float {
    return velocity.norm();
  }
  [[nodiscard]] auto getMass() const -> float { return mass; }
  [[nodiscard]] auto getRotation() const -> float { return rotation; }

  // Setters with validation
  auto setMass(float newMass) -> std::expected<void, const char *> {
    if (newMass <= 0)
      return std::unexpected("Mass must be positive");
    mass = newMass;
    return {};
  }

  auto setThrust(float newThrust) -> std::expected<void, const char *> {
    if (newThrust < 0)
      return std::unexpected("Thrust cannot be negative");
    thrust = newThrust;
    return {};
  }

  auto setBodyColor(Color color) -> void { bodyColor = color; }
  auto setThrustColor(Color color) -> void { thrustColor = color; }
};

#endif // ROCKET_HPP
