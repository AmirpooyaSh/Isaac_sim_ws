# Update the file to match the user's latest requirement format:
# - Each path should be enclosed in double quotes and end with a comma.
# - Ensure the path is concatenated in the form "~/.local/share/ov/pkg/isaac-sim-4.2.0/" + directory + ","

input_path = '/mnt/data/sub_dir.text'
output_path_final_corrected = '/mnt/data/final_corrected_with_full_path_and_commas.txt'

# Read the original content, process each line, and write to the new file
with open(input_path, 'r') as infile, open(output_path_final_corrected, 'w') as outfile:
    for line in infile:
        # Strip any existing surrounding quotes or commas, then format with the new path prefix
        directory = line.strip().strip('",')
        formatted_line = f"\"~/.local/share/ov/pkg/isaac-sim-4.2.0/{directory}\","
        outfile.write(formatted_line + '\n')

output_path_final_corrected
