# Orbit Simulation

A real-time 2D orbital mechanics simulation built with C++23, featuring interactive physics, gravitational forces, and a controllable rocket. This project demonstrates classical orbital mechanics principles with smooth graphics and intuitive controls.

![Orbit Simulation](https://img.shields.io/badge/C%2B%2B-23-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![CMake](https://img.shields.io/badge/CMake-3.25%2B-064F8C.svg)

## Features

### Physics Simulation
- **Newtonian Gravity**: Realistic gravitational force calculations using Newton's law of universal gravitation (F = GMm/r²)
- **Orbital Mechanics**: Demonstrates various orbital behaviors including circular, elliptical, and escape trajectories
- **Real-time Physics**: 60 FPS simulation with adjustable time scaling (0.25x to 16x)
- **Rocket Control**: Full 6-DOF control with thrust and rotation capabilities

### Visual Features
- **Beautiful Rendering**: Planet with atmospheric glow effects and surface details
- **Vector Visualization**: Toggle-able display of velocity (blue) and gravitational force (yellow) vectors
- **Dynamic Thrust Effects**: Animated rocket exhaust with particle-like flames
- **Smooth Camera**: Pan, zoom, and follow controls for exploring the simulation

### Interactive Controls
- **Mouse Controls**: Right-click to reposition rocket, middle-click to pan camera, scroll to zoom
- **Keyboard Controls**: Full keyboard support for rocket maneuvering and simulation parameters
- **Parameter Adjustment**: Real-time modification of gravity strength and time flow

## Prerequisites

Before building this project, ensure you have the following installed:

- **C++23 Compiler**: GCC 13+, Clang 16+, or MSVC 2022+
- **CMake**: Version 3.25 or higher
- **Git**: For cloning dependencies

### Platform-Specific Requirements

#### Linux
```bash
sudo apt-get update
sudo apt-get install build-essential cmake git
sudo apt-get install libgl1-mesa-dev libx11-dev libxrandr-dev libxi-dev
```

#### macOS
```bash
brew install cmake
xcode-select --install
```

#### Windows
- Install [Visual Studio 2022](https://visualstudio.microsoft.com/) with C++ development tools
- Install [CMake](https://cmake.org/download/)
- Ensure CMake is in your PATH

## Building the Project

### Clone the Repository
```bash
git clone https://github.com/Abhiram-Kasu/orbit_simulation.git
cd orbit_simulation
```

### Build with CMake

#### Linux/macOS
```bash
mkdir build
cd build
cmake ..
cmake --build .
```

#### Windows
```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

The executable will be generated in the `build` directory as `orbit_simulation` (or `orbit_simulation.exe` on Windows).

### Run the Simulation
```bash
# Linux/macOS
./orbit_simulation

# Windows
.\orbit_simulation.exe
```

## Usage

### Controls

#### Rocket Control
| Key | Action |
|-----|--------|
| `W` | Forward thrust |
| `S` | Reverse thrust |
| `A` | Rotate rocket counter-clockwise |
| `D` | Rotate rocket clockwise |

#### Camera Control
| Input | Action |
|-------|--------|
| `Mouse Wheel` | Zoom in/out |
| `Middle Mouse + Drag` | Pan camera |
| `Right Click` | Place rocket at cursor position with orbital velocity |

#### Simulation Control
| Key | Action |
|-----|--------|
| `UP Arrow` | Increase gravitational constant |
| `DOWN Arrow` | Decrease gravitational constant |
| `LEFT Arrow` | Decrease time scale (slow motion) |
| `RIGHT Arrow` | Increase time scale (fast forward) |
| `SPACE` | Reset time scale to 1x |
| `V` | Toggle vector visualization |
| `R` | Reset simulation to initial state |
| `ESC` | Exit simulation |

### On-Screen Information
The simulation displays real-time information:
- **Time Scale**: Current simulation speed multiplier
- **Velocity**: Rocket's velocity magnitude
- **Gravity**: Current gravitational constant
- **Zoom**: Current camera zoom level

## Project Structure

```
orbit_simulation/
├── CMakeLists.txt          # CMake build configuration
├── README.md               # This file
├── .gitignore             # Git ignore patterns
└── src/
    ├── main.cpp           # Main application and game loop
    ├── planet.hpp         # Planet class with gravity calculations
    └── rocket.hpp         # Rocket class with physics and rendering
```

## Technical Details

### Dependencies
This project automatically downloads and builds its dependencies using CMake's `FetchContent`:

- **[Raylib](https://www.raylib.com/)**: Graphics library for rendering and input
  - Version: Latest from master branch
  - Used for: Window management, 2D rendering, input handling

- **[Eigen](https://eigen.tuxfamily.org/)**: C++ template library for linear algebra
  - Version: 3.4.0
  - Used for: Vector mathematics, physics calculations

### Architecture

#### Physics Engine
The simulation uses a simple Euler integration method for physics updates:
1. Calculate gravitational force: `F = GMm/r²`
2. Apply forces to update acceleration: `a = F/m`
3. Update velocity: `v = v + a*dt`
4. Update position: `p = p + v*dt`

#### Key Classes

**Planet** (`planet.hpp`)
- Manages planet position, mass, and visual representation
- Calculates gravitational force on other objects
- Renders planet with atmospheric effects

**Rocket** (`rocket.hpp`)
- Handles rocket physics (position, velocity, acceleration)
- Provides thrust and rotation controls
- Renders rocket with dynamic thrust visualization
- Displays force vectors for debugging

### Modern C++ Features
This project showcases several C++23 features:
- `std::expected<T, E>` for error handling without exceptions
- Constexpr functions for compile-time evaluation
- `[[nodiscard]]` attributes to prevent value discarding
- Concepts and constraints for type safety
- Designated initializers for cleaner code

## Physics Background

### Orbital Mechanics
The simulation implements fundamental orbital mechanics principles:

- **Circular Orbit Velocity**: v = √(GM/r)
- **Gravitational Force**: F = GMm/r²
- **Escape Velocity**: v = √(2GM/r)

Where:
- G = Gravitational constant (adjustable in simulation)
- M = Planet mass
- m = Rocket mass
- r = Distance from planet center

### Achieving Stable Orbits
To achieve a circular orbit:
1. Position the rocket at desired orbital altitude
2. The simulation automatically calculates the required orbital velocity
3. Velocity is applied perpendicular to the planet-rocket direction
4. Fine-tune with manual thrust controls

## Tips for Exploration

1. **Circular Orbits**: Right-click to place rocket, it will start with orbital velocity
2. **Elliptical Orbits**: After placing, use thrust to increase or decrease velocity
3. **Escape Trajectory**: Apply significant forward thrust to exceed escape velocity
4. **Gravity Slingshot**: Increase planet gravity while in orbit for dramatic effects
5. **Time Manipulation**: Use slow-motion to observe fine orbital details

## Troubleshooting

### Build Issues

**CMake Error: Could not find OpenGL**
```bash
# Linux
sudo apt-get install libgl1-mesa-dev

# macOS
# OpenGL comes with Xcode Command Line Tools
```

**Compilation Error: C++23 features not supported**
- Ensure you're using a modern compiler (GCC 13+, Clang 16+, MSVC 2022+)
- Update CMake to version 3.25 or higher

### Runtime Issues

**Window doesn't open**
- Ensure graphics drivers are up to date
- Check that your system supports OpenGL

**Performance issues**
- Reduce zoom level
- Decrease time scale
- Update graphics drivers

## Future Enhancements

Potential features for future development:
- Multiple celestial bodies with n-body physics
- Fuel system with limited propellant
- Mission objectives and challenges
- Trajectory prediction and planning tools
- Save/load simulation states
- Configuration file for custom scenarios
- Additional celestial bodies (moons, asteroids)

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs, feature requests, or improvements.

## License

This project is open source. Please check the repository for license information.

## Credits

- **Raylib**: [raysan5/raylib](https://github.com/raysan5/raylib)
- **Eigen**: [libeigen/eigen](https://gitlab.com/libeigen/eigen)

## Author

Abhiram Kasu - [GitHub Profile](https://github.com/Abhiram-Kasu)

---

**Enjoy exploring orbital mechanics! 🚀🪐**
