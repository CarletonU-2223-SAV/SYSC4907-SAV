# Automated AirSim Testing

This functionality works by analyzing logs from the vehicle's logging subsystem.

Using GitHub Actions, it searches for a log with the name NH_auto_path_coords.log.
Running the simulation on a self-hosted runner is possible with this script
and is encouraged. To make installation easier, the list of dependencies can
be slimmed using the version available on [this](https://github.com/CarletonU-2223-SAV/SYSC4907-SAV/pull/45)
branch. However, the full system was never tested using these changes, so the
pull request was never merged.

The script relies on a log file being named "NH_xxx.log" or "City_xxx.log" to
identify which environment was used to run the tests.

The script should be invoked as follows: `python <path name>`. The path name
should not contain any file extension, so `python NH_auto_path_coords` suffices.

There is one optional parameter for use with GitHub Actions, `--pr-branch`.
This flag enables the script to create the output posted to GitHub as a comment.
The parameter should receive the branch name and commit hash separated by a comma
as follows: `python NH_auto_path_coords --pr-branch=my_branch,abcdef`.

The script depends on Pillow and Sympy which are already in the dependencies file.
When running on GitHub, the script also relies on images-upload-cli, which is
configured in the Actions file.
