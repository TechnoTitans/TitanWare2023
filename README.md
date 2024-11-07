# TitanWare2023
FRC 1683's primary code repository for Charged Up 2023.

![CI](https://github.com/TechnoTitans/TitanWare2023/actions/workflows/CI.yml/badge.svg)

## Structure
```
src - RobotCode
├── main - Functional code, deployed to the robot
│   ├── deploy - Files/resources deployed to the robot (not main RobotCode)
│   ├── java/frc/robot - Primary RobotCode root
└── test/java - Unit/Integration tests
    ├── frc/robot - Test classes
    └── testutils - Test utilities (if present)
```

## Contributing
For 2023, our guidelines for any and all contributions are as follows:
- Code pushed should compile (except under very specific/extreme circumstances)
- Code pushed should pass CI and all tests (if there are tests) on the local machine
- Non-working code should be properly noted as such (with a TODO)
- Keep code clean and follow the codebase conventions

### Always Keep Cooking...
1. Fix auto!!!
2. Re-fix auto!!
3. Break auto...
4. Fix auto again!
5. Repeat

### Pre-match Checklist
1. Check `CURRENT_MODE`, `COMPETITION_TYPE`
2. Deploy latest code... watch DS for events (errors, warnings, etc...)
3. Log USB fully seated in RoboRio
4. **Systems Check**
   1. `Swerve`
   2. `Vertical` & `Horizontal Elevator`
   3. `Claw Tilt` & `OpenClose`
   4. `Gyro`
   5. `PhotonVision`
5. Launch `GameNodeSelector`, ensure autos & profiles are loaded, grid is loaded
6. Battery check
7. **JOB DONE!! YIPPEE!!!**
