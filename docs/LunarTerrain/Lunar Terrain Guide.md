---
last_update:
  date: 01/14/2026
  author: Isaac Hiew Tze Yeung
---

# Lunar Terrain 

## 1.0 Introduction

The lunar terrain is obtained from Omniverse Lunar Robotics Simulator or OmniLRS (https://github.com/OmniLRS/OmniLRS). All credits goes to the creators of OmniLRS. 

In this Lunar Terrain Guide, you will learn how to install OmniLRS and use it in NVIDIA IsaacSim workstation version 5.1.0. You will be guided with the necessary installation, upgrade, and downgrade of certain dependencies.

## 2.0 Installation Guide
Ensure that you have IsaacSim workstation version 5.1.0 and ROS 2 Humble installed before running any of the bash commands below.

### 2.1 System Dependancies (GDAL)
```bash
sudo apt update
sudo apt install python3 python3-dev python3-pip gdal-bin libgdal-dev git
```

### 2.2 Clone OmniLRS
Create a folder to 
```bash
mkdir -p ~
cd ~
git clone https://github.com/OmniLRS/OmniLRS.git
```

### 2.3 Install Python Dependencies (IsaacSim Environment)
We combine the original requirements with the specific version fixes we discovered.
```bash
# Set path variable for convenience
export ISAAC_PYTHON="${HOME}/isaac-sim-5-1-0/python.sh"
cd ~/isaac-sim-5-1-0

# 1. Get System GDAL version
GDAL_VER=$(gdal-config --version)

# Verify version (should be 3.4.1)
echo $GDAL_VERSION

# 2. Install base requirements + specific fixes
$ISAAC_PYTHON -m pip install opencv-python omegaconf hydra-core skyfield zfpy \
    yamcs-client lark empy==3.3.4 numpy-quaternion gdal==$GDAL_VER

# 3. Downgrade OpenCV to support NumPy 1.x (Fixes dependency resolver error)
$ISAAC_PYTHON -m pip install "opencv-python<=4.9.0.80"

# 4. Pin Numba and Coverage to compatible versions (Fixes 'Tracer' error)
$ISAAC_PYTHON -m pip install --force-reinstall "numba==0.59.1" "llvmlite==0.42.0" "coverage==7.2.7"
```

### 2.4 Patch `run.py`
You must modify ~/OmniLRS/run.py to handle the ROS environment variables automatically. Open the file and paste the code below (this is the version that worked for you previously):

```python
__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# =================================================================================================
# START FIX FOR ISAAC SIM 5.1 / ROS 2 BRIDGE
# =================================================================================================
import os
import sys

# Define the path to the internal ROS 2 Humble libraries in Isaac Sim 5.1
# Update this path if your installation folder is different
isaac_sim_root = os.path.expanduser("~/isaac-sim-5-1-0")
bridge_lib_path = os.path.join(isaac_sim_root, "exts/isaacsim.ros2.bridge/humble/lib")

# Check if the path exists to be safe
if os.path.exists(bridge_lib_path):
    current_ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    
    # If the bridge path is not in LD_LIBRARY_PATH, add it and restart the script
    if bridge_lib_path not in current_ld_path:
        print(f"[OmniLRS Fix] Appending internal ROS bridge libs to LD_LIBRARY_PATH: {bridge_lib_path}")
        
        # Set ROS Distro to humble explicitly
        os.environ["ROS_DISTRO"] = "humble"
        os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        os.environ["LD_LIBRARY_PATH"] = f"{bridge_lib_path}:{current_ld_path}"
        
        print("[OmniLRS Fix] Restarting script with updated environment...")
        try:
            # Re-execute the script using the current python interpreter
            os.execv(sys.executable, [sys.executable] + sys.argv)
        except Exception as e:
            print(f"[OmniLRS Fix] Failed to re-exec: {e}")
            sys.exit(1)
# =================================================================================================
# END FIX
# =================================================================================================

from omegaconf import DictConfig, OmegaConf, ListConfig
from src.configurations import configFactory
from src.environments_wrappers import startSim

from typing import Dict, List
import logging
import hydra

numba_logger = logging.getLogger("numba")
numba_logger.setLevel(logging.WARNING)
matplotlib_logger = logging.getLogger("matplotlib")
matplotlib_logger.setLevel(logging.WARNING)


def resolve_tuple(*args):
    return tuple(args)


OmegaConf.register_new_resolver("as_tuple", resolve_tuple)


def omegaconfToDict(d: DictConfig) -> Dict:
    """Converts an omegaconf DictConfig to a python Dict, respecting variable interpolation.

    Args:
        d (DictConfig): OmegaConf DictConfig.

    Returns:
        Dict: Python dict."""

    if isinstance(d, DictConfig):
        ret = {}
        for k, v in d.items():
            if isinstance(v, DictConfig):
                ret[k] = omegaconfToDict(v)
            elif isinstance(v, ListConfig):
                ret[k] = [omegaconfToDict(i) for i in v]
            else:
                ret[k] = v
    elif isinstance(d, ListConfig):
        ret = [omegaconfToDict(i) for i in d]
    else:
        ret = d

    return ret


def instantiateConfigs(cfg: dict) -> dict:
    """
    Instantiates the configurations. That is if the name of the configuration is in the instantiable_configs list,
    it will create an instance of it.
    """

    instantiable_configs = configFactory.getConfigs()

    ret = {}
    for k, v in cfg.items():
        if isinstance(v, dict):
            if k in instantiable_configs:
                ret[k] = configFactory(k, **v)
            else:
                ret[k] = instantiateConfigs(v)
        else:
            ret[k] = v
    return ret


@hydra.main(config_name="config", config_path="cfg")
def run(cfg: DictConfig):
    cfg = omegaconfToDict(cfg)
    cfg = instantiateConfigs(cfg)
    SM, simulation_app = startSim(cfg)

    SM.run_simulation()
    simulation_app.close()


if __name__ == "__main__":
    run()
```

### 2.5 Generate and Save Environments
Run these commands one by one. For each command, wait for Isaac Sim to load, Stop the simulation, and File -> Save As... to ~/dssl_omniLRS_projects/.
**Generate Lunalab**
```bash
cd ~/OmniLRS

# Generate Lunalab
~/isaac-sim-5-1-0/python.sh run.py environment=lunalab
# (Once loaded: Pause -> File > Save As -> ~/dssl_omniLRS_projects/lunalab.usd)

# Generate Lunaryard 40m
~/isaac-sim-5-1-0/python.sh run.py environment=lunaryard_40m
# (Once loaded: Pause -> File > Save As -> ~/dssl_omniLRS_projects/lunaryard_40m.usd)
```

**Generate Lunaryard**
```bash
~/isaac-sim-5-1-0/python.sh run.py environment=lunaryard_20m
# Save as: lunaryard_20m_compiled.usd
```

### 2.6 Fix Textures
To ensure the saved USDs are not red when opened:
```bash
cp -r ~/OmniLRS/assets ~/dssl_omniLRS_projects/
```

