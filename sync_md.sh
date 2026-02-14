#!/bin/bash

# Add all .md files
git add *.md

# Commit if there are changes
if ! git diff-index --quiet HEAD --; then
    git commit -m "Auto-sync: $(date)"
else
    echo "No changes to commit, proceeding to sync..."
fi

# Sync
git pull --rebase origin $(git branch --show-current)
git push origin $(git branch --show-current)
