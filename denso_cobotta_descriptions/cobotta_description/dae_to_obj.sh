#!/bin/bash

# Path to assimp executable
ASSIMP_PATH="/Users/ahmed/assimp/bin/assimp"

# Check if assimp exists at the specified path
if [ ! -f "$ASSIMP_PATH" ]; then
    echo "Error: assimp not found at $ASSIMP_PATH"
    exit 1
fi

# Loop through all .dae files in the current directory
for dae_file in *.dae; do
    # Get the base filename without extension
    base_name="${dae_file%.dae}"
    
    # Output OBJ filename
    obj_file="${base_name}.obj"
    
    # Convert DAE to OBJ using assimp
    echo "Converting $dae_file to $obj_file..."
    "$ASSIMP_PATH" export "$dae_file" "$obj_file"
    
    # Check if conversion was successful
    if [ $? -eq 0 ]; then
        echo "Successfully converted $dae_file to $obj_file"
    else
        echo "Failed to convert $dae_file"
    fi
done

echo "Conversion process completed."
