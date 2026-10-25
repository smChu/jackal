#!/usr/bin/env bash
# Link repository-owned sources into an existing Clearpath workspace; no build.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${1:-$HOME/jackal_actor_ws}"
PACKAGE_SOURCE="$REPO_DIR/ros2/jackal_actor_controller"
PACKAGE_LINK="$WS/src/jackal_actor_controller"
WORLD_SOURCE="$REPO_DIR/worlds/jackal_actor_world.sdf"
WORLD_LINK="$WS/src/clearpath_simulator/clearpath_gz/worlds/jackal_actor_world.sdf"

if [[ ! -d "$(dirname "$WORLD_LINK")" ]]; then
    echo "ERROR: Clearpath world directory missing: $(dirname "$WORLD_LINK")" >&2
    echo "Import the Clearpath dependencies into the requested workspace first." >&2
    exit 1
fi

if [[ -e "$PACKAGE_LINK" || -L "$PACKAGE_LINK" ]]; then
    if [[ "$(readlink -f "$PACKAGE_LINK")" != "$PACKAGE_SOURCE" ]]; then
        echo "ERROR: Refusing to replace existing package: $PACKAGE_LINK" >&2
        exit 1
    fi
else
    ln -s "$PACKAGE_SOURCE" "$PACKAGE_LINK"
fi

if [[ "$(readlink -f "$WORLD_LINK")" != "$WORLD_SOURCE" ]]; then
    if [[ -e "$WORLD_LINK" || -L "$WORLD_LINK" ]]; then
        BACKUP="$WORLD_LINK.before_actor_controller"
        if [[ -e "$BACKUP" || -L "$BACKUP" ]]; then
            echo "ERROR: World backup already exists; inspect before relinking: $BACKUP" >&2
            exit 1
        fi
        cp -pL "$WORLD_LINK" "$BACKUP"
        echo "Original world backed up: $BACKUP"
    fi
    ln -sfn "$WORLD_SOURCE" "$WORLD_LINK"
fi

echo "Controller source: $PACKAGE_LINK -> $(readlink -f "$PACKAGE_LINK")"
echo "Clearpath world:   $WORLD_LINK -> $(readlink -f "$WORLD_LINK")"
echo "Next: source /opt/ros/humble/setup.bash"
echo "Then: cd $WS && colcon build --symlink-install --packages-select jackal_actor_controller"
echo "Then: source $WS/install/setup.bash"
echo "If the Clearpath installed world is not already a symlink, also rebuild clearpath_gz."
