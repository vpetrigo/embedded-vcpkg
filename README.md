# Embedded development with VCPKG example

## Prerequisites

- install `vcpkg-init`:

    - Linux/MacOS:
    ```
    . <(curl https://aka.ms/vcpkg-init.sh -L)
    ```
    - Windows (Powershell):
    ```
    iex (iwr -useb https://aka.ms/vcpkg-init.ps1)
    ```
    - Windows (CMD):
    ```
    curl -LO https://aka.ms/vcpkg-init.cmd && .\vcpkg-init.cmd
    ```

- activate `vcpkg-init`:

    - Linux/MacOS
    ```
    . ~/.vcpkg/vcpkg-init.sh
    ```
    - Windows (Powershell):
    ```
    . ~/.vcpkg/vcpkg-init.ps1
    ```
    - Windows (CMD)
    ```
    %USERPROFILE%\.vcpkg\vcpkg-init.cmd
    ```

- to automatically execute those scripts while starting Bash/Powershell:
    - Bash/Zsh: add the line above into the `.bashrc`/`.zshrc`
    - Windows: [PS Profile](https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_profiles?view=powershell-7.3)

## Project setup

- initialize git submodules

```bash
$ git submodule update --init --recursive
```
- copy `vcpkg-configuration.json` into the `vcpkg` directory

```bash
$ cp vcpkg-configuration.json vcpkg
```

- activate environment:

```bash
vcpkg activate --vcpkg-root=./vcpkg
```

## Add library dependency

All dependencies that are specified in the `vcpkg.json` can be installed with the following command:

```
vcpkg install --overlay-triplets=./triplet --triplet=arm-none-eabi --vcpkg-root=./vcpkg
```

## Use VCPKG with CMake

After you `activate` VCPKG environment:

- Linux/MacOS:
```bash
cmake -G Ninja -B cmake-build-debug -S . -DCMAKE_TOOLCHAIN_FILE=vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=$(readlink -f cmake/arm-gcc-toolchain.cmake)
cmake --build cmake-build-debug
```

- Windows (Powershell/CMD): unfortunatelly VCPKG artifacts environment currently does not work as expected with the Windows
```powershell
cmake -G Ninja -B cmake-build-debug -S . -DCMAKE_TOOLCHAIN_FILE="cmake\arm-gcc-toolchain.cmake"
cmake --build cmake-build-debug
```
