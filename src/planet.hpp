#ifndef PLANET_HPP
#define PLANET_HPP

#include "raylib.h"
#include <Eigen/Dense>
#include <concepts>
#include <expected>

class Planet {
private:
  Eigen::Vector2f position;
  float radius;
  float mass;
  Color color;
  Color atmosphereColor;
  float atmosphereSize;

  // Helper function to scale vector visualization
  [[nodiscard]] static constexpr auto scaleVectorSize(float magnitude)
      -> float {
    constexpr float minScale = 0.5f;
    constexpr float maxScale = 3.0f;
    constexpr float scaleFactor = 0.01f;
    return std::clamp(magnitude * scaleFactor, minScale, maxScale);
  }

public:
  struct ForceResult {
    Eigen::Vector2f force;
    float magnitude;
  };

  constexpr Planet(const Eigen::Vector2f &pos, float rad, float m) noexcept
      : position(pos), radius(rad), mass(m), color(YELLOW),
        atmosphereColor(Color{255, 255, 0, 50}), atmosphereSize(rad * 1.5f) {}

  // Calculate gravitational force on another object
  [[nodiscard]] auto
  calculateGravitationalForce(const Eigen::Vector2f &otherPos, float otherMass,
                              float G) const
      -> std::expected<ForceResult, const char *> {
    Eigen::Vector2f direction = position - otherPos;
    float distance = direction.norm();

    // Error handling for invalid parameters
    if (G <= 0)
      return std::unexpected("Invalid gravitational constant");
    if (otherMass <= 0)
      return std::unexpected("Invalid mass");

    // Prevent division by zero and extreme forces at very small distances
    distance = std::max(distance, radius);

    // Calculate force magnitude using Newton's law of universal gravitation
    float forceMagnitude = G * (mass * otherMass) / (distance * distance);
    return ForceResult{direction.normalized() * forceMagnitude, forceMagnitude};
  }

  auto draw(const Camera2D &camera) const -> void {
    // Convert world position to screen position
    Vector2 screenPos =
        GetWorldToScreen2D({position.x(), position.y()}, camera);

    const float scaledRadius = radius * camera.zoom;
    const float scaledAtmosphereSize = atmosphereSize * camera.zoom;

    // Draw atmosphere glow with dynamic opacity based on zoom
    const float atmosphereOpacity = std::clamp(0.5f / camera.zoom, 0.1f, 0.5f);
    const Color dynamicAtmosphereColor =
        ColorAlpha(atmosphereColor, atmosphereOpacity);
    DrawCircleV(screenPos, scaledAtmosphereSize, dynamicAtmosphereColor);

    // Draw planet
    DrawCircleV(screenPos, scaledRadius, color);

    // Draw surface details with dynamic sizing
    const float detailScale1 = 0.8f;
    const float detailScale2 = 0.6f;
    const float detail1 = scaledRadius * detailScale1;
    const float detail2 = scaledRadius * detailScale2;

    DrawCircleV(screenPos, detail1, ColorAlpha(ORANGE, 0.5f));
    DrawCircleV(screenPos, detail2, ColorAlpha(RED, 0.3f));

    // Draw polar regions
    const float polarRegionSize = scaledRadius * 0.3f;
    DrawCircleV({screenPos.x, screenPos.y - scaledRadius * 0.7f},
                polarRegionSize, ColorAlpha(WHITE, 0.3f));
    DrawCircleV({screenPos.x, screenPos.y + scaledRadius * 0.7f},
                polarRegionSize, ColorAlpha(WHITE, 0.3f));
  }

  // Getters - all marked [[nodiscard]] to prevent accidental value ignoring
  [[nodiscard]] auto getPosition() const -> const Eigen::Vector2f & {
    return position;
  }

  [[nodiscard]] auto getMass() const -> float { return mass; }

  [[nodiscard]] auto getRadius() const -> float { return radius; }

  // Setters with constraints checking
  auto setColor(Color newColor) -> void { color = newColor; }

  auto setMass(float newMass) -> std::expected<void, const char *> {
    if (newMass <= 0)
      return std::unexpected("Mass must be positive");
    mass = newMass;
    return {};
  }

  auto setAtmosphereColor(Color color) -> void { atmosphereColor = color; }

  auto setAtmosphereSize(float size) -> std::expected<void, const char *> {
    if (size <= radius)
      return std::unexpected("Atmosphere must be larger than planet");
    atmosphereSize = size;
    return {};
  }
};

#endif // PLANET_HPP
