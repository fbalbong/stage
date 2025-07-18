import os
import difflib

def get_all_files(directory):
    file_paths = []
    for root, _, files in os.walk(directory):
        for name in files:
            relative_path = os.path.relpath(os.path.join(root, name), start=directory)
            file_paths.append(relative_path)
    return file_paths

def compare_files(file1, file2):
    with open(file1, 'r', encoding='utf-8', errors='ignore') as f1, open(file2, 'r', encoding='utf-8', errors='ignore') as f2:
        f1_lines = f1.readlines()
        f2_lines = f2.readlines()

    return list(difflib.unified_diff(
        f1_lines,
        f2_lines,
        fromfile=file1,
        tofile=file2,
        lineterm=''
    ))

def compare_directories(dir1, dir2, output_file="diferencias.txt"):
    files1 = get_all_files(dir1)
    files2 = get_all_files(dir2)
    all_files = set(files1).union(set(files2))

    with open(output_file, 'w', encoding='utf-8') as f:
        for file in sorted(all_files):
            path1 = os.path.join(dir1, file)
            path2 = os.path.join(dir2, file)

            if os.path.exists(path1) and os.path.exists(path2):
                diff = compare_files(path1, path2)
                if diff:
                    f.write(f"\n--- Diferencias en {file} ---\n")
                    for line in diff:
                        f.write(line)
            elif os.path.exists(path1):
                f.write(f"\n--- {file} existe solo en {dir1} ---\n")
            elif os.path.exists(path2):
                f.write(f"\n--- {file} existe solo en {dir2} ---\n")

# Ejemplo de uso:
compare_directories("src_si", "src_no", "diferencias.txt")
