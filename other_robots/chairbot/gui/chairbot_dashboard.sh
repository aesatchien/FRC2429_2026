#!/usr/bin/env bash
# Starts the chairbot dashboard.  Run once with --install to put a shortcut to it on your desktop:
#     bash other_robots/chairbot/gui/chairbot_dashboard.sh --install

ENV_NAME="${ROBO_ENV:-robo2026}"
GUI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

fail() {  # the window closes when this script ends, so wait until the message has been read
    echo "ERROR: $1"
    read -rp "Press Enter to close..."
    exit 1
}

env_works() {  # does this folder hold a python that can actually run the dashboard?
    [ -x "$1/python.exe" ] &&
        PATH="$1:$1/Library/bin:$1/Scripts:$PATH" "$1/python.exe" -c "import PyQt6.QtWidgets, ntcore, wpimath" 2>/dev/null
}

find_env() {  # prints the folder of the first working environment, or returns failure
    local user_home app_data registry line root dir
    local candidates=()
    user_home="$(cygpath -u "$USERPROFILE")"
    app_data="$(cygpath -u "$LOCALAPPDATA")"

    # conda and mamba both record every environment they create here
    registry="$user_home/.conda/environments.txt"
    if [ -f "$registry" ]; then
        while IFS= read -r line; do
            line="${line%$'\r'}"
            if [ "${line##*[\\/]}" = "$ENV_NAME" ]; then
                candidates+=("$(cygpath -u "$line")")
            fi
        done < "$registry"
    fi

    # otherwise try the places they usually get installed
    for root in "$MAMBA_ROOT_PREFIX" "$user_home/.local/share/mamba" "$user_home/.conda" \
                "$user_home/miniforge3" "$user_home/mambaforge" "$user_home/miniconda3" "$user_home/anaconda3" \
                "$app_data/miniforge3" "$app_data/mambaforge" "$app_data/miniconda3" "$app_data/anaconda3" \
                "/c/ProgramData/miniforge3" "/c/ProgramData/miniconda3" "/c/ProgramData/anaconda3"; do
        candidates+=("$root/envs/$ENV_NAME")
    done

    for dir in "${candidates[@]}"; do
        if env_works "$dir"; then
            echo "$dir"
            return 0
        fi
    done
    return 1
}

install_shortcut() {
    local desktop="$1"
    if [ -z "$desktop" ]; then  # ask windows, because OneDrive can move the desktop
        desktop="$(powershell.exe -NoProfile -Command "[Environment]::GetFolderPath('Desktop')" | tr -d '\r')"
    fi
    find_env > /dev/null || fail "no working '$ENV_NAME' environment found, so the shortcut would not run"

    # only windows can write a .lnk file, so hand this one step to powershell - env vars avoid quoting trouble
    LNK_PATH="$(cygpath -w "$desktop")\\Chairbot Dashboard.lnk" \
    LNK_TARGET="$(cygpath -w /git-bash.exe)" \
    LNK_ARGS="--no-cd \"$(cygpath -w "$GUI_DIR/chairbot_dashboard.sh")\"" \
    LNK_ICON="$(cygpath -w "$GUI_DIR/chairbot.ico")" \
    powershell.exe -NoProfile -Command '
        $shortcut = (New-Object -ComObject WScript.Shell).CreateShortcut($env:LNK_PATH)
        $shortcut.TargetPath = $env:LNK_TARGET
        $shortcut.Arguments = $env:LNK_ARGS
        $shortcut.IconLocation = $env:LNK_ICON
        $shortcut.Save()
        Write-Output "Created $env:LNK_PATH"
    ' || fail "could not create the shortcut"
}

if [ "$1" = "--install" ]; then
    install_shortcut "$2"
    exit 0
fi

ENV_DIR="$(find_env)" || fail "no working '$ENV_NAME' environment found"
echo "Using $ENV_DIR"
export PATH="$ENV_DIR:$ENV_DIR/Library/bin:$ENV_DIR/Scripts:$PATH"
cd "$GUI_DIR" || fail "could not open $GUI_DIR"  # dashboard.py loads its .ui and images from the current folder
python.exe -u dashboard.py || fail "the dashboard stopped with an error"
