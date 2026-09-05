from pathlib import Path
import platform
import subprocess
import sys
import venv


PROJECT_ROOT = Path(__file__).resolve().parent
VENV_DIR = PROJECT_ROOT / ".venv"
REQUIREMENTS_FILE = PROJECT_ROOT / "requirements.txt"


def get_venv_python():
    """Return the Python executable inside the virtual environment."""

    if platform.system() == "Windows":
        return VENV_DIR / "Scripts" / "python.exe"

    return VENV_DIR / "bin" / "python"


def run(command):
    """Run a command and stop if it fails."""

    print()
    print(">", " ".join(str(item) for item in command))
    subprocess.check_call(command)


def main():
    system = platform.system()

    print("=" * 60)
    print("Python Environment Setup")
    print("=" * 60)

    print(f"Operating system : {system}")
    print(f"System Python    : {sys.executable}")
    print(f"Python version   : {platform.python_version()}")
    print(f"Project folder   : {PROJECT_ROOT}")
    print(f"Virtual env      : {VENV_DIR}")

    # ---------------------------------------------------------
    # Create virtual environment if it doesn't already exist
    # ---------------------------------------------------------

    if not VENV_DIR.exists():
        print()
        print("Creating virtual environment...")

        venv.create(
            VENV_DIR,
            with_pip=True
        )

        print("Virtual environment created.")

    else:
        print()
        print("Virtual environment already exists.")

    python = get_venv_python()

    if not python.exists():
        raise RuntimeError(
            f"Virtual environment exists but Python was not found at:\n"
            f"{python}\n\n"
            f"Delete the .venv folder and run this script again."
        )

    # ---------------------------------------------------------
    # Upgrade pip
    # ---------------------------------------------------------

    print()
    print("Updating pip...")

    run([
        str(python),
        "-m",
        "pip",
        "install",
        "--upgrade",
        "pip"
    ])

    # ---------------------------------------------------------
    # Install/update dependencies
    # ---------------------------------------------------------

    if REQUIREMENTS_FILE.exists():

        print()
        print("Installing project dependencies...")

        run([
            str(python),
            "-m",
            "pip",
            "install",
            "-r",
            str(REQUIREMENTS_FILE)
        ])

    else:

        print()
        print("WARNING: requirements.txt was not found.")
        print("No project dependencies were installed.")

    # ---------------------------------------------------------
    # Finished
    # ---------------------------------------------------------

    print()
    print("=" * 60)
    print("Environment ready!")
    print("=" * 60)

    if system == "Windows":
        print()
        print("Activate with:")
        print(r"    .venv\Scripts\activate")

    else:
        print()
        print("Activate with:")
        print("    source .venv/bin/activate")

    print()
    print("Or simply configure your IDE to use:")

    if system == "Windows":
        print(r"    .venv\Scripts\python.exe")
    else:
        print("    .venv/bin/python")

    print()


if __name__ == "__main__":
    main()