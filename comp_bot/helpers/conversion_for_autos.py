import json
import os


def flip_path_data(input_path, output_path=None):
    # Load the JSON data
    with open(input_path, 'r') as f:
        data = json.load(f)

    # 1. Update waypoints (anchor, prevControl, nextControl)
    for wp in data.get('waypoints', []):
        for key in ['anchor', 'prevControl', 'nextControl']:
            if wp[key] is not None:
                wp[key]['y'] = 8.05 - wp[key]['y']

    # 2. Update rotation targets
    for target in data.get('rotationTargets', []):
        if 'rotationDegrees' in target:
            target['rotationDegrees'] *= -1

    # 3. Update goal end state rotation
    if 'goalEndState' in data and 'rotation' in data['goalEndState']:
        data['goalEndState']['rotation'] *= -1

    # 4. Update ideal starting state rotation
    if 'idealStartingState' in data and 'rotation' in data['idealStartingState']:
        data['idealStartingState']['rotation'] *= -1

    # Determine output filename if not provided
    if output_path is None:
        base, ext = os.path.splitext(input_path)
        output_path = f"{base}_flipped{ext}"

    # Save the new JSON
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"Success! Flipped file saved to: {output_path}")


# Usage
if __name__ == "__main__":
    path = "Right_Bump_Destruction.path"  # Replace with your actual file path
    if os.path.exists(path):
        flip_path_data(path)
    else:
        print(f"Error: Could not find file at {path}")
