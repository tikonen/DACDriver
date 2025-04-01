import subprocess
import sys
import os
import shutil


def main():

    if len(sys.argv) < 2:
        print("Prepare firmware")
        print("")
        print("Usage: %s <firmware.bin>" % (sys.argv[0]))
        exit(0)

    filename = sys.argv[1]

    name = os.path.basename(filename)
    (name, ext) = os.path.splitext(name)

    # Get the current git commit hash
    gitp = subprocess.check_output(
        ["git", "rev-parse", "--short", "HEAD"], stderr=subprocess.STDOUT)
    # Decode and strip any extra whitespace
    git_sha = gitp.decode("utf-8").strip()

    outfilename = f'{name}-{git_sha}{ext}'

    shutil.copy(filename, os.path.join('Binaries', outfilename))
    print(f"{filename} -> {outfilename}")


if __name__ == "__main__":
    main()
