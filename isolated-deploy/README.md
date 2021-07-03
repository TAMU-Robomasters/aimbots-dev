# Standalone programming of the MCB via Openocd

## Prerequisites

- Download the latest version of [openocd](https://gnutoolchains.com/arm-eabi/openocd/). Download the newest version of the software and extract the files to a permanent location. The downloaded file is of type `.7zip`, so you will need a way to extract 7zip files, either by installing the 7zip extractor or by using an online unzipper.
- Add the path of the `/bin` directory of OpenOCD to your PATH environment variable.
    - In your Start menu, search for "environment" and select "Edit the system environment variables."

    - In the "User variables for _{your username here}_", where _{your username here}_ is a placeholder for your actual name. Double click on "Path."<br><br>
    <img src="https://gitlab.com/aruw/controls/aruw-mcb/uploads/c02e3c34b52f4c5756cdd9149e891da6/EnvironmentVarsMain.jpg" height="500px">

    - In "Edit environment variable," click "New."<br><br>
    <img src="https://gitlab.com/aruw/controls/aruw-mcb/uploads/44ba293d6876ca856ad51289c3bf2368/EnvironmentVarsPathVar.jpg" height="500px">

    - Add the absolute path of the `/bin` directory of OpenOCD to the box that shows up after you click "New."<br><br>
    <img src="https://gitlab.com/aruw/controls/aruw-mcb/uploads/8f25bbfc917288f9c92155f64b76d476/EnvironmentVarsPathAddition.jpg" height="500px">

    - Select "OK".
- Download the ST-Link V-2 driver [here](https://drive.google.com/drive/u/1/folders/1Ndk8Q-uUtzo3sQtzOoguDAVBZSM3IKT1), unzip, and run the executable to install the driver.

## How to deploy to the MCB

- Download the weekly build artifact from the develop branch (see [here](https://docs.gitlab.com/ee/ci/pipelines/job_artifacts.html#downloading-the-latest-artifacts) if you don't know how to do this). This artifact includes this readme as well as `openocd.cfg` and a number of `.elf` files.
- In a terminal (bash or windows based), navigate to the folder containing the downloaded artifacts.
- Type the following:
    ```
    openocd -f "./openocd.cfg" -c "modm_program path/to/elf/file/elf-file-name.elf"
    ```
   where `path/to/elf/file` is a valid path to an `.elf` file (either absolute or relative to where you are in the terminal), `elf-file-name.elf` is a valid `.elf` file (presumably one of the ones you downloaded from the build artifact), and `./openocd.cfg` is the file you downloaded in the first step. You should replace `./openocd.cfg` with the actual name of the file you downloaded if the `.cfg` file has been changed.

## Creating an elf file

To get a `.elf` file, you must build the codebase. Refer to [the main repo](#how-to-build-code-and-program-the-mcb) if you are confused about how to build. If you don't have the build environment set up but need a new build, ask on slack for someone to send you a `.elf` file. If you do have the environment but want to deploy using this technique, once you have a build, find the file in the build directory `build/hardware/<debug/hardware>/template-project.elf`. This is the `.elf` file you will want to use while deploying to the MCB.