import os
import difflib

def compare_directories(dir1, dir2, output_path):
    diff_results = []
    
    for root, _, files in os.walk(dir1):
        for file in files:
            rel_path = os.path.relpath(os.path.join(root, file), dir1)
            file1_path = os.path.join(dir1, rel_path)
            file2_path = os.path.join(dir2, rel_path)
            
            if os.path.exists(file2_path):
                with open(file1_path, 'r', errors='ignore') as f1, open(file2_path, 'r', errors='ignore') as f2:
                    f1_lines = f1.readlines()
                    f2_lines = f2.readlines()
                    diff = list(difflib.unified_diff(f1_lines, f2_lines, fromfile=file1_path, tofile=file2_path))
                    if diff:
                        diff_results.extend(diff)
            else:
                diff_results.append(f'--- {file1_path}\n+++ {file2_path} (MISSING)\n')

    with open(output_path, 'w', encoding='utf-8') as out_file:
        out_file.writelines(diff_results)
    
    return output_path