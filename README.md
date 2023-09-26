# TitanWare2023
FRC 1683's primarily code repository for Charged Up 2023.

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
