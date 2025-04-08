import re

cnt = 0
def group_if_statements(file_path):
    # Read the file
    with open(file_path, 'r') as file:
        content = file.read()

    # Regular expression to detect outer if blocks, allowing for multi-line function arguments
    outer_if_pattern = re.compile(
        r'(\s*if\s*\(\s*/\*.*?\*/\s*(sphere_environment_in_collision|sphere_sphere_self_collision)\([\s\S]*?\)\s*\)\s*\{)([\s\S]*?)(\s*\}\s*//.*?$)',
        re.MULTILINE
    )

    # Regular expression to match consecutive inner if-statements inside an outer block
    inner_if_pattern = re.compile(
        r'\s*if\s*\(\s*(sphere_environment_in_collision|sphere_sphere_self_collision)\(([\s\S]*?)\)\s*\)\s*\{\s*\n\s*return\s+false;\s*\n\s*\}',
        re.MULTILINE
    )

    def merge_inner_if_statements(inner_block):
        """Groups inner if-statements with 'collision = true;' inside an outer block."""
        # Find all matching if-statements inside the current block
        matches = inner_if_pattern.findall(inner_block)

        if not matches:
            return inner_block  # Return unchanged if no matches found

        # Extract function name (should be the same within each block)
        function_name = matches[0][0]  # Either 'sphere_environment_in_collision' or 'sphere_sphere_self_collision'

        # Format each condition correctly, ensuring no newline after '('
        merged_conditions = " ||\n    ".join(
            f"{function_name}({m[1].strip()})" for m in matches
        )

        merged_if_statement = f"    if ({merged_conditions}) {{\n        collision = true;\n    }}"

        # Remove original inner if-statements and insert merged statement
        return inner_if_pattern.sub("", inner_block).strip() + "\n" + merged_if_statement

    # Process each outer if block separately
    def process_outer_block(match):

        outer_if_start = match.group(1)  # The `if` statement for the outer block
        inner_block = match.group(3)  # The inner if-statements and other content
        outer_if_end = match.group(4)  # Closing `}` of the outer block

        # Process the inner block to group if-statements
        formatted_inner_block = merge_inner_if_statements(inner_block)

        # Early exit statement after the outer block
        global cnt
        early_exit = f"\n    printf(\"here{cnt}\\n\");\n    collision = __any_sync(0xFFFFFFFF, collision);\n    __syncwarp();\n    if (collision) {{\n        printf(\"here{cnt} collision\\n\");\n        return false;\n    }}"
        cnt += 1
        # Reassemble the entire block
        return f"{outer_if_start}\n{formatted_inner_block}\n{outer_if_end}{early_exit}"

    # Apply transformation to all outer if blocks
    formatted_content = outer_if_pattern.sub(process_outer_block, content)

    # Write back to the file
    with open("fkcc_fetch_collapsed.cu", 'w') as file:
        file.write(formatted_content)

    print("File formatted successfully!")

# Example usage
file_path = "temp_fetch.cuh"  # Change this to your actual file
group_if_statements(file_path)