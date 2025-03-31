# find . -maxdepth 1 -type d | while read dir; do count=$(find "$dir" -type f | wc -l); echo "$dir: $count files"; done

find ./data/ -type d | while read dir; do count=$(find "$dir" -maxdepth 1 -type f | wc -l); echo "$dir: $count files"; done

