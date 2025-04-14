#!/bin/bash

# Base directory to save the EuRoC sequences
BASE_DIR="${HOME}/ros2_test/Datasets/EuRoC"
mkdir -p "$BASE_DIR"
cd "$BASE_DIR" || exit

# Dataset base URL
BASE_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"

# Declare an associative array: folder_name => zip_name
declare -A SEQUENCES=(
  ["MH01"]="machine_hall/MH_01_easy/MH_01_easy.zip"
  ["MH02"]="machine_hall/MH_02_easy/MH_02_easy.zip"
  ["MH03"]="machine_hall/MH_03_medium/MH_03_medium.zip"
  ["MH04"]="machine_hall/MH_04_difficult/MH_04_difficult.zip"
  ["MH05"]="machine_hall/MH_05_difficult/MH_05_difficult.zip"
  ["V101"]="vicon_room1/V1_01_easy/V1_01_easy.zip"
  ["V102"]="vicon_room1/V1_02_medium/V1_02_medium.zip"
  ["V103"]="vicon_room1/V1_03_difficult/V1_03_difficult.zip"
  ["V201"]="vicon_room2/V2_01_easy/V2_01_easy.zip"
  ["V202"]="vicon_room2/V2_02_medium/V2_02_medium.zip"
  ["V203"]="vicon_room2/V2_03_difficult/V2_03_difficult.zip"
)

# Loop through and download each sequence
for seq_name in "${!SEQUENCES[@]}"; do
  ZIP_PATH="${SEQUENCES[$seq_name]}"
  ZIP_NAME=$(basename "$ZIP_PATH")
  FOLDER_NAME="${seq_name}"

  echo "Downloading $ZIP_NAME..."
  wget "$BASE_URL/$ZIP_PATH" -O "$ZIP_NAME"

  echo "Extracting $ZIP_NAME into $FOLDER_NAME..."
  mkdir -p "$FOLDER_NAME"
  unzip -q "$ZIP_NAME" -d "$FOLDER_NAME"

  echo "Cleaning up $ZIP_NAME..."
  rm "$ZIP_NAME"
  echo "Done with $FOLDER_NAME."
  echo ""
done

echo "All EuRoC sequences downloaded and extracted to $BASE_DIR"
