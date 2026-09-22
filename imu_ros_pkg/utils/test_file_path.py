from pathlib import Path
import sys
from pprint import pprint

print("System imported modules: \n", sys.modules)

# print(Path(__file__).resolve().parents[1])

# --------- USING ABS PATH ---------
# log_dir = Path(__file__).resolve().parents[1] / ("log_data")
# print(log_dir)
# file_latest = max(log_dir.iterdir(), key=lambda f: f.stat().st_mtime)
# print(file_latest, file_latest.name)

# --------- USING REL PATH ---------
print("Current folder: ", Path.cwd())
log_dir_rel = Path('..') / "log_data"
print(log_dir_rel)
# print(log_dir_rel.resolve())
file_latest_rel = max(log_dir_rel.iterdir(), key=lambda f: f.stat().st_mtime)
print(file_latest_rel, file_latest_rel.name)


print('-- dirs by iter --')
for i in log_dir_rel.iterdir():
    print(i, type(i)) 
# print(max([]))