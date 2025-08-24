#!/usr/bin/env python3
import argparse, json, os
DEFAULT = os.path.join("artifacts","semantic_mock.json")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--db", default=DEFAULT)
    args = ap.parse_args()

    with open(args.db,'r') as f:
        db = json.load(f)
    label = args.label.strip().lower()
    pose = db.get(label)
    if pose is None:
        print(f"Label '{label}' not found.")
    else:
        print(f"{label}: x={pose[0]}, y={pose[1]}, theta={pose[2]}")

if __name__=="__main__":
    main()
