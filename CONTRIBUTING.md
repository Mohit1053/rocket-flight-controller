# Contributing to Predicones

Thank you for your interest in contributing to Predicones! This document provides guidelines for contributing to this rocket flight controller project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [How to Contribute](#how-to-contribute)
- [Development Setup](#development-setup)
- [Coding Standards](#coding-standards)
- [Commit Guidelines](#commit-guidelines)
- [Pull Request Process](#pull-request-process)
- [Safety Guidelines](#safety-guidelines)

## Code of Conduct

By participating in this project, you agree to maintain a respectful and inclusive environment.

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork**:
   ```bash
   git clone https://github.com/YOUR-USERNAME/predicones.git
   cd predicones
   ```
3. **Add upstream remote**:
   ```bash
   git remote add upstream https://github.com/ORIGINAL-OWNER/predicones.git
   ```

## How to Contribute

### Reporting Bugs

- Check existing issues first
- Include detailed steps to reproduce
- Provide hardware configuration details
- Include relevant log output

### Suggesting Features

- Open an issue with the `enhancement` label
- Describe the feature and use case
- Explain how it would improve flight performance

### Contributing Code

1. Create a feature branch
2. Make your changes
3. Test thoroughly (GROUND TEST ONLY)
4. Submit a pull request

## Development Setup

### Prerequisites

- Arduino IDE 2.0+ or PlatformIO
- STM32 board support package
- Required libraries installed

### Local Setup

1. Install Arduino IDE or PlatformIO
2. Install STM32 board support
3. Install required libraries:
   - Adafruit_BMP3XX
   - MPU9250
   - SCServo
   - SdFat
4. Connect your hardware
5. Select correct board and port
6. Upload and test

## Coding Standards

### C++ Style

- Use meaningful variable names
- Comment complex algorithms
- Follow Arduino naming conventions
- Keep functions focused and small

### Example

```cpp
/**
 * Calculate fin deflection using LQR control
 * @param roll Current roll angle (radians)
 * @param pitch Current pitch angle (radians)
 * @return Servo position delta
 */
int calculateFinDeflection(float roll, float pitch) {
    // LQR gain application
    float u = K[0] * roll + K[1] * pitch;
    return constrain((int)(u * SCALE_U), -MAX_DELTA, MAX_DELTA);
}
```

### Documentation

- Add docstrings to functions
- Update README for new features
- Include units in variable names where helpful

## Commit Guidelines

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <description>
```

### Types

- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `refactor`: Code refactoring
- `test`: Adding tests
- `chore`: Maintenance

### Examples

```
feat(fins): add adaptive gain scheduling
fix(deployment): correct altitude threshold logic
docs(readme): update wiring diagram
```

## Pull Request Process

1. **Test thoroughly** on hardware (ground tests only)
2. **Update documentation** for any changes
3. **Update CHANGELOG.md**
4. **Fill out PR template** completely

### PR Checklist

- [ ] Code compiles without warnings
- [ ] Ground tested on actual hardware
- [ ] Documentation updated
- [ ] CHANGELOG.md updated
- [ ] No hardware-specific paths in code

## Safety Guidelines

### Critical Rules

⚠️ **This is flight-critical software. Follow these rules:**

1. **NEVER test with live pyrotechnics**
2. **Always ground test before flight**
3. **Use simulation mode for development**
4. **Verify sensor readings before flight**
5. **Test deployment mechanisms separately**
6. **Follow all local rocketry regulations**

### Testing Requirements

- All changes must be ground tested
- Document test procedure and results
- Never rush testing for a launch deadline
- Have backup firmware ready

---

Thank you for contributing to safer, more capable model rocketry! 🚀
